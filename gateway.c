#include <stdio_ext.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <termios.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/timerfd.h>
#include <sys/poll.h>
#include <sys/wait.h>
#include <time.h>

#define SERIAL_DEVICE "/dev/ttyS1"

#define SOCKET_PATH   "/tmp/scada.sock"
#define DB_FILE       "/tmp/process.db"
#define LOG_FILE      "/tmp/scada_alarms.log"

#define SLAVE_ADDRESS 0x01

#define MODBUS_FUNC_READ_HOLDING 0x03
#define MODBUS_FUNC_WRITE_REG    0x06
#define MODBUS_FUNC_WRITE_COIL   0x05

#define REG_PRESSURE             0
#define REG_FLOW                 1
#define REG_TEMPERATURE          2
#define REG_STATUS               6

#define POLL_INTERVAL_MS         100
#define MAX_RETRIES              3

#define DEVICE_ONLINE            1
#define DEVICE_OFFLINE           0

#define STATUS_VALVE_BIT         (1 << 0)
#define STATUS_ALARM_BIT         (1 << 1)
#define STATUS_OFFLINE_BIT       (1 << 2)

volatile sig_atomic_t running = 1;

int serial_fd = -1;
int socket_fd = -1;
int timer_fd = -1;
int db_fd = -1;

int pipe_fd[2];

pid_t alarm_pid = -1;

typedef struct
{
    uint16_t holding_registers[10];

    uint8_t coil_state;

    uint8_t device_online;

    uint32_t last_poll_time;

    uint32_t poll_count;

    uint32_t crc_errors;

} ProcessDB;

typedef struct
{
    char tag[16];

    uint16_t value;

    uint16_t limit;

    uint8_t severity;

    uint32_t timestamp;

} AlarmEvent;

ProcessDB *process_db;

uint16_t CRC16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for(uint16_t pos = 0; pos < length; pos++)
    {
        crc ^= data[pos];

        for(int i = 0; i < 8; i++)
        {
            if(crc & 1)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

void SignalHandler(int sig)
{
    running = 0;
}

void ConfigureSerial(void)
{
    serial_fd = open(SERIAL_DEVICE, O_RDWR | O_NOCTTY);

    if(serial_fd < 0)
    {
        perror("open serial");
        exit(EXIT_FAILURE);
    }

    struct termios tty;

    memset(&tty, 0, sizeof(tty));

    if(tcgetattr(serial_fd, &tty) != 0)
    {
        perror("tcgetattr");
        exit(EXIT_FAILURE);
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;

    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tty.c_cflag |= (CLOCAL | CREAD);

    tty.c_cflag &= ~PARENB;

    tty.c_cflag &= ~CSTOPB;

    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN] = 0;

    tty.c_cc[VTIME] = 1;

    if(tcsetattr(serial_fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr");
        exit(EXIT_FAILURE);
    }
}

void CreateDatabase(void)
{
    db_fd = open(DB_FILE, O_RDWR | O_CREAT, 0666);

    if(db_fd < 0)
    {
        perror("open db");
        exit(EXIT_FAILURE);
    }

    if(ftruncate(db_fd, sizeof(ProcessDB)) < 0)
    {
        perror("ftruncate");
        exit(EXIT_FAILURE);
    }

    process_db = mmap(NULL,
                      sizeof(ProcessDB),
                      PROT_READ | PROT_WRITE,
                      MAP_SHARED,
                      db_fd,
                      0);

    if(process_db == MAP_FAILED)
    {
        perror("mmap");
        exit(EXIT_FAILURE);
    }

    memset(process_db, 0, sizeof(ProcessDB));
}

void SetupTimer(void)
{
    timer_fd = timerfd_create(CLOCK_MONOTONIC, 0);

    if(timer_fd < 0)
    {
        perror("timerfd_create");
        exit(EXIT_FAILURE);
    }

    struct itimerspec timerSpec;

    timerSpec.it_interval.tv_sec = 0;
    timerSpec.it_interval.tv_nsec = 100000000;

    timerSpec.it_value.tv_sec = 0;
    timerSpec.it_value.tv_nsec = 100000000;

    if(timerfd_settime(timer_fd, 0, &timerSpec, NULL) < 0)
    {
        perror("timerfd_settime");
        exit(EXIT_FAILURE);
    }
}

void CreateSocket(void)
{
    socket_fd = socket(AF_UNIX, SOCK_STREAM, 0);

    if(socket_fd < 0)
    {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    struct sockaddr_un addr;

    memset(&addr, 0, sizeof(addr));

    addr.sun_family = AF_UNIX;

    strncpy(addr.sun_path,
            SOCKET_PATH,
            sizeof(addr.sun_path) - 1);

    unlink(SOCKET_PATH);

    if(bind(socket_fd,
            (struct sockaddr *)&addr,
            sizeof(addr)) < 0)
    {
        perror("bind");
        exit(EXIT_FAILURE);
    }

    if(listen(socket_fd, 5) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
}

void SendAlarm(const char *tag,
               uint16_t value,
               uint16_t limit,
               uint8_t severity)
{
    AlarmEvent event;

    memset(&event, 0, sizeof(event));

    strncpy(event.tag,
            tag,
            sizeof(event.tag) - 1);

    event.value = value;

    event.limit = limit;

    event.severity = severity;

    event.timestamp = time(NULL);

    write(pipe_fd[1], &event, sizeof(event));
}

void AlarmProcess(void)
{
    AlarmEvent event;

    while(1)
    {
        ssize_t bytes = read(pipe_fd[0],
                             &event,
                             sizeof(event));

        if(bytes <= 0)
        {
            continue;
        }

        printf("ALARM: %s value=%u limit=%u severity=%u\n",
               event.tag,
               event.value,
               event.limit,
               event.severity);

        FILE *logFile = fopen(LOG_FILE, "a");

        if(logFile)
        {
            fprintf(logFile,
                    "%u %s %u %u %u\n",
                    event.timestamp,
                    event.tag,
                    event.value,
                    event.limit,
                    event.severity);

            fclose(logFile);
        }
    }
}

void StartAlarmProcess(void)
{
    if(pipe(pipe_fd) < 0)
    {
        perror("pipe");
        exit(EXIT_FAILURE);
    }

    alarm_pid = fork();

    if(alarm_pid < 0)
    {
        perror("fork");
        exit(EXIT_FAILURE);
    }

    if(alarm_pid == 0)
    {
        AlarmProcess();

        exit(0);
    }

    close(pipe_fd[0]);
}

int PollModbus(void)
{
    uint8_t request[8];

    uint8_t response[64];

    struct flock lock;

    request[0] = SLAVE_ADDRESS;
    request[1] = MODBUS_FUNC_READ_HOLDING;
    request[2] = 0x00;
    request[3] = 0x00;
    request[4] = 0x00;
    request[5] = 0x03;

    uint16_t crc = CRC16(request, 6);

    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;

    tcflush(serial_fd, TCIFLUSH);

    write(serial_fd, request, 8);

    usleep(100000);

    ssize_t len = read(serial_fd,
                       response,
                       sizeof(response));

    if(len < 11)
    {
        process_db->crc_errors++;

        return -1;
    }

    uint16_t received_crc =
        response[len - 2] |
        (response[len - 1] << 8);

    uint16_t calculated_crc =
        CRC16(response, len - 2);

    if(received_crc != calculated_crc)
    {
        process_db->crc_errors++;

        return -1;
    }

    lock.l_type = F_WRLCK;
    lock.l_whence = SEEK_SET;
    lock.l_start = 0;
    lock.l_len = sizeof(ProcessDB);

    fcntl(db_fd, F_SETLKW, &lock);

    process_db->holding_registers[REG_PRESSURE] =
        (response[3] << 8) | response[4];

    process_db->holding_registers[REG_FLOW] =
        (response[5] << 8) | response[6];

    process_db->holding_registers[REG_TEMPERATURE] =
        (response[7] << 8) | response[8];

    process_db->device_online = DEVICE_ONLINE;

    process_db->last_poll_time = time(NULL);

    process_db->poll_count++;

    msync(process_db,
          sizeof(ProcessDB),
          MS_SYNC);

    lock.l_type = F_UNLCK;

    fcntl(db_fd, F_SETLK, &lock);

    if(process_db->holding_registers[REG_PRESSURE] > 3600)
    {
        SendAlarm("PRESSURE",
                  process_db->holding_registers[REG_PRESSURE],
                  3600,
                  2);
    }

    if(process_db->holding_registers[REG_FLOW] > 9000)
    {
        SendAlarm("FLOW",
                  process_db->holding_registers[REG_FLOW],
                  9000,
                  2);
    }

    if(process_db->holding_registers[REG_TEMPERATURE] > 1200)
    {
        SendAlarm("TEMPERATURE",
                  process_db->holding_registers[REG_TEMPERATURE],
                  1200,
                  3);
    }

    return 0;
}

void HandleHMIClient(void)
{
    int client_fd;

    char buffer[128];

    client_fd = accept(socket_fd, NULL, NULL);

    if(client_fd < 0)
    {
        return;
    }

    ssize_t len = read(client_fd,
                       buffer,
                       sizeof(buffer) - 1);

    if(len <= 0)
    {
        close(client_fd);

        return;
    }

    buffer[len] = '\0';

    char response[128];

    if(strncmp(buffer, "PRESSURE", 8) == 0)
    {
        snprintf(response,
                 sizeof(response),
                 "PRESSURE=%u bar\n",
                 process_db->holding_registers[REG_PRESSURE] / 10);
    }
    else if(strncmp(buffer, "FLOW", 4) == 0)
    {
        snprintf(response,
                 sizeof(response),
                 "FLOW=%u L/min\n",
                 process_db->holding_registers[REG_FLOW] / 10);
    }
    else if(strncmp(buffer, "TEMPERATURE", 11) == 0)
    {
        snprintf(response,
                 sizeof(response),
                 "TEMPERATURE=%u C\n",
                 process_db->holding_registers[REG_TEMPERATURE] / 10);
    }
    else
    {
        snprintf(response,
                 sizeof(response),
                 "UNKNOWN TAG\n");
    }

    write(client_fd,
          response,
          strlen(response));

    close(client_fd);
}

void Cleanup(void)
{
    msync(process_db,
          sizeof(ProcessDB),
          MS_SYNC);

    munmap(process_db,
           sizeof(ProcessDB));

    close(serial_fd);

    close(socket_fd);

    close(timer_fd);

    close(db_fd);

    unlink(SOCKET_PATH);
}

int main(void)
{
    signal(SIGINT, SignalHandler);

    ConfigureSerial();

    CreateDatabase();

    SetupTimer();

    CreateSocket();

    StartAlarmProcess();

    struct pollfd fds[2];

    fds[0].fd = timer_fd;
    fds[0].events = POLLIN;

    fds[1].fd = socket_fd;
    fds[1].events = POLLIN;

    int missedPolls = 0;

    while(running)
    {
        int ret = poll(fds, 2, -1);

        if(ret < 0)
        {
            if(errno == EINTR)
            {
                continue;
            }

            perror("poll");

            break;
        }

        if(fds[0].revents & POLLIN)
        {
            uint64_t expirations;

            read(timer_fd,
                 &expirations,
                 sizeof(expirations));

            if(PollModbus() == 0)
            {
                missedPolls = 0;
            }
            else
            {
                missedPolls++;

                if(missedPolls >= MAX_RETRIES)
                {
                    process_db->device_online =
                        DEVICE_OFFLINE;

                    process_db->holding_registers[REG_STATUS]
                        |= STATUS_OFFLINE_BIT;
                }
            }
        }

        if(fds[1].revents & POLLIN)
        {
            HandleHMIClient();
        }
    }

    Cleanup();

    return 0;
}
