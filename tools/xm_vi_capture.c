#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

/*
 * XM530 VI ioctls taken from OpenXiongmai mpi_vi.c.
 */
#define XM_VI_INIT          0x00005606u
#define XM_VI_GET_FRAME     0xC0185604u
#define XM_VI_RELEASE_FRAME 0x40185605u

/*
 * MMZ range observed on the current XM530 camera:
 *
 *   [0x82300000--------0x88000000]
 *
 * The range check can be disabled with --no-mmz-check if another
 * firmware uses a different MMZ layout.
 */
#define DEFAULT_MMZ_START 0x82300000u
#define DEFAULT_MMZ_END   0x88000000u

#define DEFAULT_CHANNEL       0u
#define DEFAULT_RETRIES      30u
#define DEFAULT_DELAY_MS     100u
#define MAX_CAPTURE_WIDTH   8192u
#define MAX_CAPTURE_HEIGHT  8192u

typedef struct {
    uint32_t chn;
    uint32_t pool_id;
    uint32_t phys_addr_y;
    uint32_t phys_addr_c;
    uint32_t width;
    uint32_t height;
} vi_frame_t;

typedef struct {
    void *map_base;
    unsigned char *data;
    size_t map_length;
    uint32_t physical_address;
    size_t data_length;
} mapped_plane_t;

typedef struct {
    const char *output_path;
    const char *vi_device;

    uint32_t channel;
    uint32_t retries;
    uint32_t delay_ms;

    uint32_t width_override;
    uint32_t height_override;

    uint32_t mmz_start;
    uint32_t mmz_end;

    int initialize_vi;
    int metadata_only;
    int show_proc_status;
    int mmz_check;
    int save_combined;
    int save_separate;
    int quiet;
} options_t;

static volatile sig_atomic_t stop_requested = 0;


/* -------------------------------------------------------------------------
 * Signal handling
 * ------------------------------------------------------------------------- */

static void signal_handler(int signum)
{
    (void)signum;
    stop_requested = 1;
}


/* -------------------------------------------------------------------------
 * Generic helpers
 * ------------------------------------------------------------------------- */

static void sleep_ms(uint32_t milliseconds)
{
    struct timespec request;
    struct timespec remaining;

    request.tv_sec = milliseconds / 1000u;
    request.tv_nsec =
        (long)(milliseconds % 1000u) * 1000000L;

    while (nanosleep(&request, &remaining) != 0) {
        if (errno != EINTR)
            break;

        if (stop_requested)
            break;

        request = remaining;
    }
}


static int parse_u32(const char *text, uint32_t *value)
{
    char *end = NULL;
    unsigned long parsed;

    if (text == NULL || value == NULL || *text == '\0')
        return -1;

    errno = 0;
    parsed = strtoul(text, &end, 0);

    if (errno != 0 ||
        end == text ||
        *end != '\0' ||
        parsed > UINT32_MAX) {
        return -1;
    }

    *value = (uint32_t)parsed;
    return 0;
}


static int write_all(int fd, const void *buffer, size_t size)
{
    const unsigned char *data =
        (const unsigned char *)buffer;

    size_t done = 0;

    while (done < size) {
        ssize_t result =
            write(fd, data + done, size - done);

        if (result < 0) {
            if (errno == EINTR)
                continue;

            return -1;
        }

        if (result == 0) {
            errno = EIO;
            return -1;
        }

        done += (size_t)result;
    }

    return 0;
}


static int write_file(const char *path,
                      const void *buffer,
                      size_t size)
{
    int fd;
    int result = -1;

    fd = open(path,
              O_WRONLY | O_CREAT | O_TRUNC,
              0644);

    if (fd < 0) {
        fprintf(stderr,
                "Cannot create %s: %s\n",
                path,
                strerror(errno));
        return -1;
    }

    if (write_all(fd, buffer, size) != 0) {
        fprintf(stderr,
                "Cannot write %s: %s\n",
                path,
                strerror(errno));
        goto cleanup;
    }

    if (fsync(fd) != 0) {
        fprintf(stderr,
                "Cannot sync %s: %s\n",
                path,
                strerror(errno));
        goto cleanup;
    }

    result = 0;

cleanup:
    close(fd);
    return result;
}


static void show_vi_proc_status(const char *heading)
{
    FILE *fp;
    char line[512];

    printf("\n========================================\n");
    printf(" %s\n", heading);
    printf("========================================\n");

    fp = fopen("/proc/umap/vi", "r");

    if (fp == NULL) {
        printf("Unable to read /proc/umap/vi: %s\n",
               strerror(errno));
        return;
    }

    while (fgets(line, sizeof(line), fp) != NULL)
        fputs(line, stdout);

    fclose(fp);
}


/* -------------------------------------------------------------------------
 * Frame validation
 * ------------------------------------------------------------------------- */

static int frame_has_metadata(const vi_frame_t *frame)
{
    if (frame == NULL)
        return 0;

    if (frame->phys_addr_y != 0)
        return 1;

    if (frame->phys_addr_c != 0)
        return 1;

    if (frame->width != 0)
        return 1;

    if (frame->height != 0)
        return 1;

    return 0;
}


static int frame_is_complete(const vi_frame_t *frame)
{
    if (frame == NULL)
        return 0;

    if (frame->phys_addr_y == 0 ||
        frame->phys_addr_c == 0 ||
        frame->width == 0 ||
        frame->height == 0) {
        return 0;
    }

    return 1;
}


static int valid_dimensions(uint32_t width,
                            uint32_t height)
{
    if (width == 0 || height == 0)
        return 0;

    if (width > MAX_CAPTURE_WIDTH ||
        height > MAX_CAPTURE_HEIGHT) {
        return 0;
    }

    return 1;
}


static int valid_mmz_range(uint32_t address,
                           size_t size,
                           uint32_t mmz_start,
                           uint32_t mmz_end)
{
    uint64_t start = address;
    uint64_t end = start + (uint64_t)size;

    if (start < (uint64_t)mmz_start)
        return 0;

    if (end > (uint64_t)mmz_end)
        return 0;

    if (end < start)
        return 0;

    return 1;
}


/* -------------------------------------------------------------------------
 * MMZ mapping through /dev/mem
 * ------------------------------------------------------------------------- */

static int map_plane(int mem_fd,
                     uint32_t physical_address,
                     size_t plane_size,
                     mapped_plane_t *plane)
{
    long page_size;

    uint32_t physical_page;
    size_t page_offset;
    size_t required_size;
    size_t mapped_size;

    void *mapped;

    if (plane == NULL || plane_size == 0) {
        errno = EINVAL;
        return -1;
    }

    memset(plane, 0, sizeof(*plane));

    page_size = sysconf(_SC_PAGESIZE);

    if (page_size <= 0) {
        fprintf(stderr,
                "Cannot determine page size\n");
        return -1;
    }

    if ((page_size & (page_size - 1)) != 0) {
        fprintf(stderr,
                "Unsupported page size: %ld\n",
                page_size);
        return -1;
    }

    physical_page =
        physical_address &
        ~((uint32_t)page_size - 1u);

    page_offset =
        (size_t)(physical_address - physical_page);

    if (plane_size > SIZE_MAX - page_offset) {
        fprintf(stderr,
                "MMZ mapping size overflow\n");
        return -1;
    }

    required_size = page_offset + plane_size;

    if (required_size >
        SIZE_MAX - ((size_t)page_size - 1u)) {
        fprintf(stderr,
                "MMZ aligned mapping size overflow\n");
        return -1;
    }

    mapped_size =
        (required_size + (size_t)page_size - 1u) &
        ~((size_t)page_size - 1u);

    errno = 0;

    /*
     * OpenXiongmai XM_MPI_SYS_Mmap() uses:
     *
     *   open("/dev/mem", O_RDWR | O_SYNC)
     *   mmap(..., PROT_READ | PROT_WRITE, MAP_SHARED, ...)
     *
     * The capture utility follows those permissions for compatibility
     * with the old XM530 kernel, but never writes into the MMZ mapping.
     */
    mapped = mmap(NULL,
                  mapped_size,
                  PROT_READ | PROT_WRITE,
                  MAP_SHARED,
                  mem_fd,
                  (off_t)physical_page);

    if (mapped == MAP_FAILED) {
        fprintf(stderr,
                "mmap failed for physical address "
                "0x%08" PRIX32 ", size=%zu: %s\n",
                physical_address,
                plane_size,
                strerror(errno));
        return -1;
    }

    plane->map_base = mapped;
    plane->map_length = mapped_size;
    plane->data =
        (unsigned char *)mapped + page_offset;

    plane->physical_address = physical_address;
    plane->data_length = plane_size;

    return 0;
}


static void unmap_plane(mapped_plane_t *plane)
{
    if (plane == NULL)
        return;

    if (plane->map_base != NULL &&
        plane->map_base != MAP_FAILED &&
        plane->map_length != 0) {
        munmap(plane->map_base,
               plane->map_length);
    }

    memset(plane, 0, sizeof(*plane));
}


/* -------------------------------------------------------------------------
 * VI ioctl operations
 * ------------------------------------------------------------------------- */

static int initialize_vi_handle(int vi_fd)
{
    int result;
    int saved_errno;

    errno = 0;
    result = ioctl(vi_fd, XM_VI_INIT, 0);
    saved_errno = errno;

    printf("VI INIT ioctl: result=%d errno=%d (%s)\n",
           result,
           saved_errno,
           strerror(saved_errno));

    if (result != 0) {
        errno = saved_errno;
        return -1;
    }

    return 0;
}


static int get_frame_once(int vi_fd,
                          uint32_t channel,
                          vi_frame_t *frame,
                          unsigned int attempt)
{
    int result;
    int saved_errno;

    if (frame == NULL) {
        errno = EINVAL;
        return -1;
    }

    memset(frame, 0, sizeof(*frame));
    frame->chn = channel;

    errno = 0;
    result = ioctl(vi_fd,
                   XM_VI_GET_FRAME,
                   frame);

    saved_errno = errno;

    printf("GetFrame attempt %u: "
           "result=%d errno=%d (%s), "
           "pool=%" PRIu32 ", "
           "Y=0x%08" PRIX32 ", "
           "C=0x%08" PRIX32 ", "
           "%ux%u\n",
           attempt,
           result,
           saved_errno,
           strerror(saved_errno),
           frame->pool_id,
           frame->phys_addr_y,
           frame->phys_addr_c,
           frame->width,
           frame->height);

    if (result != 0) {
        errno = saved_errno;
        return -1;
    }

    return 0;
}


static int acquire_frame(int vi_fd,
                         const options_t *options,
                         vi_frame_t *frame)
{
    uint32_t attempt;

    for (attempt = 1;
         attempt <= options->retries &&
         !stop_requested;
         attempt++) {
        int ioctl_ok;

        ioctl_ok =
            get_frame_once(vi_fd,
                           options->channel,
                           frame,
                           attempt);

        /*
         * A zero return value with all-zero metadata was observed on
         * the XM530. Treat it as "no frame returned yet" and retry.
         */
        if (ioctl_ok == 0 &&
            frame_is_complete(frame)) {
            return 0;
        }

        /*
         * If the driver returned partial metadata, keep it visible in
         * the log but do not map or release invalid physical addresses.
         */
        if (frame_has_metadata(frame) &&
            !frame_is_complete(frame)) {
            printf("WARNING: VI returned incomplete frame metadata.\n");
        }

        if (attempt < options->retries &&
            options->delay_ms != 0) {
            sleep_ms(options->delay_ms);
        }
    }

    return -1;
}


static int release_frame(int vi_fd,
                         vi_frame_t *frame)
{
    int result;
    int saved_errno;

    if (vi_fd < 0 || frame == NULL) {
        errno = EINVAL;
        return -1;
    }

    errno = 0;
    result = ioctl(vi_fd,
                   XM_VI_RELEASE_FRAME,
                   frame);

    saved_errno = errno;

    printf("VI ReleaseFrame ioctl: "
           "result=%d errno=%d (%s)\n",
           result,
           saved_errno,
           strerror(saved_errno));

    if (result != 0) {
        errno = saved_errno;
        return -1;
    }

    return 0;
}


/* -------------------------------------------------------------------------
 * Output paths
 * ------------------------------------------------------------------------- */

static int make_plane_path(const char *combined_path,
                           const char *suffix,
                           char *buffer,
                           size_t buffer_size)
{
    int result;

    result = snprintf(buffer,
                      buffer_size,
                      "%s.%s",
                      combined_path,
                      suffix);

    if (result < 0 ||
        (size_t)result >= buffer_size) {
        return -1;
    }

    return 0;
}


/* -------------------------------------------------------------------------
 * Command line
 * ------------------------------------------------------------------------- */

static void print_usage(const char *program)
{
    printf(
        "XM530 direct VI frame capture\n\n"

        "Usage:\n"
        "  %s [options]\n\n"

        "Options:\n"
        "  -o, --output <file>\n"
        "      Combined SP420 output file.\n"
        "      Default: /tmp/vi_frame.yuv\n\n"

        "  -c, --channel <number>\n"
        "      VI channel to query.\n"
        "      Default: 0\n\n"

        "  -r, --retries <number>\n"
        "      Number of GetFrame attempts.\n"
        "      Default: 30\n\n"

        "  -d, --delay-ms <milliseconds>\n"
        "      Delay between GetFrame attempts.\n"
        "      Default: 100\n\n"

        "  --dangerous-reset-vi\n"
        "      Execute VI ioctl 0x5606.\n"
        "      WARNING: this resets the global VI configuration.\n"
        "      Do NOT use while Majestic is running.\n\n"
        
        "  --vi-device <path>\n"
        "      VI device or /proc/<pid>/fd/<fd> path.\n"
        "      Default: /dev/vi\n\n"
        
        "  --metadata-only\n"
        "      Run VI ioctls and display frame metadata, but do not map\n"
        "      or save the returned frame.\n\n"

        "  --separate\n"
        "      Save separate files in addition to the combined file:\n"
        "        <output>.Y\n"
        "        <output>.C\n\n"

        "  --separate-only\n"
        "      Save only separate Y and C planes.\n\n"

        "  --width <pixels>\n"
        "      Override the width returned by the VI driver.\n\n"

        "  --height <pixels>\n"
        "      Override the height returned by the VI driver.\n\n"

        "  --no-mmz-check\n"
        "      Disable the physical MMZ safety-range validation.\n\n"

        "  --mmz-start <address>\n"
        "      Override MMZ physical start address.\n"
        "      Default: 0x%08X\n\n"

        "  --mmz-end <address>\n"
        "      Override MMZ physical end address.\n"
        "      Default: 0x%08X\n\n"

        "  --show-proc\n"
        "      Print /proc/umap/vi before and after capture.\n\n"

        "  -q, --quiet\n"
        "      Reduce non-essential output.\n\n"

        "  -h, --help\n"
        "      Show this help.\n\n"

        "Examples:\n"
        "  %s\n\n"

        "  %s --show-proc\n\n"

        "  %s --metadata-only --retries 50 --delay-ms 100\n\n"

        "  %s --no-init --metadata-only\n\n"

        "  %s -c 1 --metadata-only\n\n"

        "  %s -o /tmp/channel0.yuv --separate\n\n"

        "  %s --width 1920 --height 1080 "
        "-o /tmp/forced.yuv\n",

        program,
        DEFAULT_MMZ_START,
        DEFAULT_MMZ_END,
        program,
        program,
        program,
        program,
        program,
        program,
        program);
}


static int parse_options(int argc,
                         char **argv,
                         options_t *options)
{
    int i;

    if (options == NULL)
        return -1;

    memset(options, 0, sizeof(*options));

    options->output_path =
        "/tmp/vi_frame.yuv";
    options->vi_device =
        "/dev/vi";
    
    options->channel =
        DEFAULT_CHANNEL;

    options->retries =
        DEFAULT_RETRIES;

    options->delay_ms =
        DEFAULT_DELAY_MS;

    options->mmz_start =
        DEFAULT_MMZ_START;

    options->mmz_end =
        DEFAULT_MMZ_END;

    options->initialize_vi = 0;
    options->mmz_check = 1;
    options->save_combined = 1;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-h") ||
            !strcmp(argv[i], "--help")) {
            print_usage(argv[0]);
            exit(0);

        } else if (!strcmp(argv[i], "-o") ||
                   !strcmp(argv[i], "--output")) {
            if (++i >= argc) {
                fprintf(stderr,
                        "Missing output filename\n");
                return -1;
            }

            options->output_path = argv[i];

        } else if (!strcmp(argv[i], "-c") ||
                   !strcmp(argv[i], "--channel")) {
            if (++i >= argc ||
                parse_u32(argv[i],
                          &options->channel) != 0) {
                fprintf(stderr,
                        "Invalid VI channel\n");
                return -1;
            }

        } else if (!strcmp(argv[i], "-r") ||
                   !strcmp(argv[i], "--retries")) {
            if (++i >= argc ||
                parse_u32(argv[i],
                          &options->retries) != 0 ||
                options->retries == 0) {
                fprintf(stderr,
                        "Invalid retry count\n");
                return -1;
            }

        } else if (!strcmp(argv[i], "-d") ||
                   !strcmp(argv[i], "--delay-ms")) {
            if (++i >= argc ||
                parse_u32(argv[i],
                          &options->delay_ms) != 0) {
                fprintf(stderr,
                        "Invalid delay\n");
                return -1;
            }
        } else if (!strcmp(argv[i], "--vi-device")) {
            if (++i >= argc) {
                fprintf(stderr,
                        "Missing VI device path\n");
                return -1;
            }

            options->vi_device = argv[i];

        } else if (!strcmp(argv[i], "--dangerous-reset-vi")) {
            options->initialize_vi = 1;

        } else if (!strcmp(argv[i],
                           "--metadata-only")) {
            options->metadata_only = 1;

        } else if (!strcmp(argv[i], "--separate")) {
            options->save_separate = 1;

        } else if (!strcmp(argv[i],
                           "--separate-only")) {
            options->save_separate = 1;
            options->save_combined = 0;

        } else if (!strcmp(argv[i], "--width")) {
            if (++i >= argc ||
                parse_u32(argv[i],
                          &options->width_override) != 0) {
                fprintf(stderr,
                        "Invalid width override\n");
                return -1;
            }

        } else if (!strcmp(argv[i], "--height")) {
            if (++i >= argc ||
                parse_u32(argv[i],
                          &options->height_override) != 0) {
                fprintf(stderr,
                        "Invalid height override\n");
                return -1;
            }

        } else if (!strcmp(argv[i],
                           "--no-mmz-check")) {
            options->mmz_check = 0;

        } else if (!strcmp(argv[i],
                           "--mmz-start")) {
            if (++i >= argc ||
                parse_u32(argv[i],
                          &options->mmz_start) != 0) {
                fprintf(stderr,
                        "Invalid MMZ start address\n");
                return -1;
            }

        } else if (!strcmp(argv[i],
                           "--mmz-end")) {
            if (++i >= argc ||
                parse_u32(argv[i],
                          &options->mmz_end) != 0) {
                fprintf(stderr,
                        "Invalid MMZ end address\n");
                return -1;
            }

        } else if (!strcmp(argv[i],
                           "--show-proc")) {
            options->show_proc_status = 1;

        } else if (!strcmp(argv[i], "-q") ||
                   !strcmp(argv[i], "--quiet")) {
            options->quiet = 1;

        } else {
            fprintf(stderr,
                    "Unknown option: %s\n",
                    argv[i]);
            return -1;
        }
    }

    if (options->mmz_start >= options->mmz_end) {
        fprintf(stderr,
                "MMZ start must be lower than MMZ end\n");
        return -1;
    }

    if (options->channel > 31u) {
        fprintf(stderr,
                "Unreasonable VI channel: %u\n",
                options->channel);
        return -1;
    }

    return 0;
}


/* -------------------------------------------------------------------------
 * Main
 * ------------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    options_t options;
    vi_frame_t frame;

    mapped_plane_t y_plane;
    mapped_plane_t c_plane;

    int vi_fd = -1;
    int mem_fd = -1;
    int output_fd = -1;

    int frame_acquired = 0;
    int exit_status = 1;

    uint32_t effective_width;
    uint32_t effective_height;

    uint64_t pixel_count;

    size_t y_size;
    size_t c_size;
    size_t total_size;

    int64_t plane_distance;

    char y_path[512];
    char c_path[512];

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    memset(&frame, 0, sizeof(frame));
    memset(&y_plane, 0, sizeof(y_plane));
    memset(&c_plane, 0, sizeof(c_plane));

    if (parse_options(argc,
                      argv,
                      &options) != 0) {
        print_usage(argv[0]);
        return 1;
    }

    printf("========================================\n");
    printf(" XM530 VI FRAME CAPTURE / DEBUG TOOL\n");
    printf("========================================\n");

    printf("VI channel     : %u\n",
           options.channel);
    
    printf("VI device      : %s\n",
       options.vi_device);
  
    printf("Output         : %s\n",
           options.output_path);

    printf("Retries        : %u\n",
           options.retries);

    printf("Retry delay    : %u ms\n",
           options.delay_ms);

    printf("VI INIT        : %s\n",
           options.initialize_vi ?
               "enabled" : "disabled");

    printf("Metadata only  : %s\n",
           options.metadata_only ?
               "yes" : "no");

    printf("MMZ validation : %s\n",
           options.mmz_check ?
               "enabled" : "disabled");

    if (options.mmz_check) {
        printf("MMZ range      : "
               "0x%08" PRIX32
               " - 0x%08" PRIX32 "\n",
               options.mmz_start,
               options.mmz_end);
    }

    if (options.show_proc_status)
        show_vi_proc_status("VI STATUS BEFORE CAPTURE");

    vi_fd = open(options.vi_device, O_RDONLY);

    if (vi_fd < 0) {
        fprintf(stderr,
                "Cannot open %s: %s\n",
                options.vi_device,
                strerror(errno));
        goto cleanup;
    }

    printf("\nOpened %s successfully, fd=%d\n",
           options.vi_device,
           vi_fd);

    if (options.initialize_vi) {
        printf("\nInitializing VI handle...\n");

        if (initialize_vi_handle(vi_fd) != 0) {
            fprintf(stderr,
                    "VI initialization failed\n");
            goto cleanup;
        }
    }

    printf("\nRequesting frame...\n");

    if (acquire_frame(vi_fd,
                      &options,
                      &frame) != 0) {
        fprintf(stderr,
                "\nNo complete VI frame was returned "
                "after %u attempt(s).\n",
                options.retries);

        fprintf(stderr,
                "Try diagnostics such as:\n"
                "  %s --metadata-only --show-proc\n"
                "  %s --vi-device /proc/<pid>/fd/<fd> "
                "--metadata-only --show-proc\n"
                "  %s -c 1 --metadata-only --show-proc\n",
                argv[0],
                argv[0],
                argv[0]);

        goto cleanup;
    }

    /*
     * GetFrame succeeded and returned complete metadata.
     * From this point onward the frame must be released.
     */
    frame_acquired = 1;

    printf("\n========================================\n");
    printf(" FRAME RETURNED BY VI\n");
    printf("========================================\n");

    printf("Pool ID        : %" PRIu32 "\n",
           frame.pool_id);

    printf("Y physical     : 0x%08" PRIX32 "\n",
           frame.phys_addr_y);

    printf("C physical     : 0x%08" PRIX32 "\n",
           frame.phys_addr_c);

    printf("Driver width   : %" PRIu32 "\n",
           frame.width);

    printf("Driver height  : %" PRIu32 "\n",
           frame.height);

    plane_distance =
        (int64_t)(uint64_t)frame.phys_addr_c -
        (int64_t)(uint64_t)frame.phys_addr_y;

    printf("C-Y distance   : %" PRId64
           " bytes (0x%" PRIX64 ")\n",
           plane_distance,
           (uint64_t)plane_distance);

    effective_width =
        options.width_override != 0 ?
            options.width_override :
            frame.width;

    effective_height =
        options.height_override != 0 ?
            options.height_override :
            frame.height;

    printf("Effective width: %" PRIu32 "\n",
           effective_width);

    printf("Effective height: %" PRIu32 "\n",
           effective_height);

    if (!valid_dimensions(effective_width,
                          effective_height)) {
        fprintf(stderr,
                "Invalid effective dimensions\n");
        goto cleanup;
    }

    pixel_count =
        (uint64_t)effective_width *
        (uint64_t)effective_height;

    if (pixel_count > SIZE_MAX) {
        fprintf(stderr,
                "Frame size exceeds addressable memory\n");
        goto cleanup;
    }

    y_size = (size_t)pixel_count;
    c_size = (size_t)(pixel_count / 2u);

    if (y_size > SIZE_MAX - c_size) {
        fprintf(stderr,
                "Combined frame size overflow\n");
        goto cleanup;
    }

    total_size = y_size + c_size;

    printf("\nExpected SP420 layout:\n");

    printf("Y size          : %zu bytes\n",
           y_size);

    printf("C size          : %zu bytes\n",
           c_size);

    printf("Combined size   : %zu bytes\n",
           total_size);

    if (plane_distance > 0 &&
        (uint64_t)plane_distance !=
            (uint64_t)y_size) {
        printf("\nWARNING:\n");
        printf("The C-Y address distance differs from "
               "width*height.\n");
        printf("This can indicate stride, alignment, padding, "
               "or separate MMZ allocations.\n");
    }

    if (options.metadata_only) {
        printf("\nMetadata-only mode selected. "
               "No MMZ data will be mapped or written.\n");

        exit_status = 0;
        goto cleanup;
    }

    if (options.mmz_check) {
        if (!valid_mmz_range(frame.phys_addr_y,
                             y_size,
                             options.mmz_start,
                             options.mmz_end)) {
            fprintf(stderr,
                    "Y plane is outside the configured MMZ range\n");
            goto cleanup;
        }

        if (!valid_mmz_range(frame.phys_addr_c,
                             c_size,
                             options.mmz_start,
                             options.mmz_end)) {
            fprintf(stderr,
                    "C plane is outside the configured MMZ range\n");
            goto cleanup;
        }
    }

    mem_fd = open("/dev/mem",
                  O_RDWR | O_SYNC);

    if (mem_fd < 0) {
        fprintf(stderr,
                "Cannot open /dev/mem: %s\n",
                strerror(errno));
        goto cleanup;
    }

    printf("\nOpened /dev/mem successfully, fd=%d\n",
           mem_fd);

    if (map_plane(mem_fd,
                  frame.phys_addr_y,
                  y_size,
                  &y_plane) != 0) {
        goto cleanup;
    }

    printf("Mapped Y plane: "
           "phys=0x%08" PRIX32
           ", bytes=%zu, map=%p\n",
           y_plane.physical_address,
           y_plane.data_length,
           y_plane.map_base);

    if (map_plane(mem_fd,
                  frame.phys_addr_c,
                  c_size,
                  &c_plane) != 0) {
        goto cleanup;
    }

    printf("Mapped C plane: "
           "phys=0x%08" PRIX32
           ", bytes=%zu, map=%p\n",
           c_plane.physical_address,
           c_plane.data_length,
           c_plane.map_base);

    if (options.save_combined) {
        output_fd =
            open(options.output_path,
                 O_WRONLY | O_CREAT | O_TRUNC,
                 0644);

        if (output_fd < 0) {
            fprintf(stderr,
                    "Cannot create %s: %s\n",
                    options.output_path,
                    strerror(errno));
            goto cleanup;
        }

        printf("\nWriting combined SP420 file...\n");

        if (write_all(output_fd,
                      y_plane.data,
                      y_size) != 0) {
            fprintf(stderr,
                    "Failed to write Y data: %s\n",
                    strerror(errno));
            goto cleanup;
        }

        if (write_all(output_fd,
                      c_plane.data,
                      c_size) != 0) {
            fprintf(stderr,
                    "Failed to write C data: %s\n",
                    strerror(errno));
            goto cleanup;
        }

        if (fsync(output_fd) != 0) {
            fprintf(stderr,
                    "Failed to sync output file: %s\n",
                    strerror(errno));
            goto cleanup;
        }

        close(output_fd);
        output_fd = -1;

        printf("Combined file   : %s\n",
               options.output_path);

        printf("Combined bytes  : %zu\n",
               total_size);
    }

    if (options.save_separate) {
        if (make_plane_path(options.output_path,
                            "Y",
                            y_path,
                            sizeof(y_path)) != 0 ||
            make_plane_path(options.output_path,
                            "C",
                            c_path,
                            sizeof(c_path)) != 0) {
            fprintf(stderr,
                    "Separate-plane filename is too long\n");
            goto cleanup;
        }

        printf("\nWriting separate planes...\n");

        if (write_file(y_path,
                       y_plane.data,
                       y_size) != 0) {
            goto cleanup;
        }

        if (write_file(c_path,
                       c_plane.data,
                       c_size) != 0) {
            goto cleanup;
        }

        printf("Y plane file    : %s\n", y_path);
        printf("C plane file    : %s\n", c_path);
    }

    exit_status = 0;

cleanup:
    if (output_fd >= 0)
        close(output_fd);

    unmap_plane(&c_plane);
    unmap_plane(&y_plane);

    if (mem_fd >= 0)
        close(mem_fd);

    /*
     * Release only a frame that was successfully acquired with complete
     * metadata. Do not issue ReleaseFrame for the previously observed
     * all-zero response.
     */
    if (frame_acquired) {
        printf("\nReleasing VI frame...\n");

        if (release_frame(vi_fd,
                          &frame) != 0) {
            fprintf(stderr,
                    "WARNING: unable to release VI frame\n");
            exit_status = 1;
        }
    }

    if (vi_fd >= 0)
        close(vi_fd);

    if (options.show_proc_status)
        show_vi_proc_status("VI STATUS AFTER CAPTURE");

    printf("\n========================================\n");
    printf(" RESULT\n");
    printf("========================================\n");

    printf("%s\n",
           exit_status == 0 ?
               "OK" : "FAILED");

    return exit_status;
}
