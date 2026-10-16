#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

/*
 * XM530 VI ioctls, taken from OpenXiongmai mpi_vi.c.
 */
#define XM_VI_GET_FRAME     0xC0185604u
#define XM_VI_RELEASE_FRAME 0x40185605u

/*
 * Known MMZ range from /proc/umap/mmz.
 *
 * This is used only as a safety/plausibility check.
 */
#define MMZ_PHYS_START 0x82300000u
#define MMZ_PHYS_END   0x88000000u

/*
 * Conservative limits against invalid driver replies.
 */
#define MAX_CAPTURE_WIDTH  8192u
#define MAX_CAPTURE_HEIGHT 8192u

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
} mapped_plane_t;

static int write_all(int fd, const void *buffer, size_t size)
{
    const unsigned char *data = buffer;
    size_t done = 0;

    while (done < size) {
        ssize_t result = write(fd, data + done, size - done);

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

static int valid_mmz_range(uint32_t address, size_t size)
{
    uint64_t start = address;
    uint64_t end = start + size;

    if (start < MMZ_PHYS_START)
        return 0;

    if (end > MMZ_PHYS_END)
        return 0;

    if (end < start)
        return 0;

    return 1;
}

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
        fprintf(stderr, "Cannot determine system page size\n");
        return -1;
    }

    /*
     * The XM530 code uses 4 KiB alignment. Verify that the runtime
     * page size is compatible.
     */
    if ((page_size & (page_size - 1)) != 0) {
        fprintf(stderr,
                "Unsupported non-power-of-two page size: %ld\n",
                page_size);
        return -1;
    }

    physical_page =
        physical_address & ~((uint32_t)page_size - 1u);

    page_offset =
        (size_t)(physical_address - physical_page);

    if (plane_size > SIZE_MAX - page_offset) {
        fprintf(stderr, "Mapping size overflow\n");
        return -1;
    }

    required_size = page_offset + plane_size;

    mapped_size =
        (required_size + (size_t)page_size - 1u) &
        ~((size_t)page_size - 1u);

    /*
     * The existing XM_MPI_SYS_Mmap implementation opens /dev/mem with
     * O_RDWR|O_SYNC and maps with PROT_READ|PROT_WRITE|MAP_SHARED.
     *
     * This program never writes to the mapped MMZ memory, but uses the
     * same mapping permissions for compatibility with the old kernel.
     */
    mapped = mmap(NULL,
                  mapped_size,
                  PROT_READ | PROT_WRITE,
                  MAP_SHARED,
                  mem_fd,
                  (off_t)physical_page);

    if (mapped == MAP_FAILED) {
        fprintf(stderr,
                "mmap failed for physical address 0x%08X: %s\n",
                physical_address,
                strerror(errno));
        return -1;
    }

    plane->map_base = mapped;
    plane->map_length = mapped_size;
    plane->data =
        (unsigned char *)mapped + page_offset;

    return 0;
}

static void unmap_plane(mapped_plane_t *plane)
{
    if (plane == NULL)
        return;

    if (plane->map_base != NULL &&
        plane->map_base != MAP_FAILED &&
        plane->map_length != 0) {
        munmap(plane->map_base, plane->map_length);
    }

    memset(plane, 0, sizeof(*plane));
}

static int release_frame(int vi_fd, vi_frame_t *frame)
{
    int result;

    if (vi_fd < 0 || frame == NULL)
        return -1;

    errno = 0;
    result = ioctl(vi_fd, XM_VI_RELEASE_FRAME, frame);

    if (result != 0) {
        fprintf(stderr,
                "VI ReleaseFrame failed: result=%d, errno=%d (%s)\n",
                result,
                errno,
                strerror(errno));
        return -1;
    }

    return 0;
}

static void print_usage(const char *program)
{
    fprintf(stderr,
            "Usage:\n"
            "  %s [output-file] [channel]\n\n"
            "Examples:\n"
            "  %s /tmp/vi_frame.yuv\n"
            "  %s /tmp/vi_frame.yuv 0\n\n"
            "The output consists of:\n"
            "  1. Y plane, width * height bytes\n"
            "  2. Interleaved chroma plane, width * height / 2 bytes\n\n"
            "The SP420 chroma order may be NV12 (UV) or NV21 (VU).\n",
            program,
            program,
            program);
}

int main(int argc, char **argv)
{
    const char *output_path = "/tmp/vi_frame.yuv";

    unsigned long parsed_channel = 0;
    char *end = NULL;

    int vi_fd = -1;
    int mem_fd = -1;
    int out_fd = -1;

    int frame_acquired = 0;
    int exit_status = 1;

    vi_frame_t frame;

    mapped_plane_t y_plane;
    mapped_plane_t c_plane;

    size_t y_size;
    size_t c_size;
    size_t total_size;

    uint64_t pixel_count;
    int64_t plane_distance;

    memset(&frame, 0, sizeof(frame));
    memset(&y_plane, 0, sizeof(y_plane));
    memset(&c_plane, 0, sizeof(c_plane));

    if (argc > 3) {
        print_usage(argv[0]);
        return 1;
    }

    if (argc >= 2)
        output_path = argv[1];

    if (argc >= 3) {
        errno = 0;
        parsed_channel = strtoul(argv[2], &end, 0);

        if (errno != 0 ||
            end == argv[2] ||
            *end != '\0' ||
            parsed_channel > UINT32_MAX) {
            fprintf(stderr,
                    "Invalid VI channel: %s\n",
                    argv[2]);
            return 1;
        }
    }

    frame.chn = (uint32_t)parsed_channel;

    printf("========================================\n");
    printf(" XM530 VI FRAME CAPTURE\n");
    printf("========================================\n");

    printf("VI channel : %u\n", frame.chn);
    printf("Output     : %s\n", output_path);
    printf("Access     : VI ioctl + read-only MMZ copy\n");

    vi_fd = open("/dev/vi", O_RDONLY);

    if (vi_fd < 0) {
        fprintf(stderr,
                "Cannot open /dev/vi: %s\n",
                strerror(errno));
        goto cleanup;
    }

    errno = 0;

    {
        int result = ioctl(vi_fd, XM_VI_GET_FRAME, &frame);

        if (result != 0) {
            fprintf(stderr,
                    "VI GetFrame failed: result=%d, errno=%d (%s)\n",
                    result,
                    errno,
                    strerror(errno));
            goto cleanup;
        }
    }

    frame_acquired = 1;

    printf("\nFrame returned by VI:\n");
    printf("Pool ID    : %u\n", frame.pool_id);
    printf("Y physical : 0x%08X\n", frame.phys_addr_y);
    printf("C physical : 0x%08X\n", frame.phys_addr_c);
    printf("Width      : %u\n", frame.width);
    printf("Height     : %u\n", frame.height);

    plane_distance =
        (int64_t)(uint64_t)frame.phys_addr_c -
        (int64_t)(uint64_t)frame.phys_addr_y;

    printf("C-Y offset : %lld bytes (0x%llX)\n",
           (long long)plane_distance,
           (unsigned long long)plane_distance);

    if (frame.width == 0 ||
        frame.height == 0 ||
        frame.width > MAX_CAPTURE_WIDTH ||
        frame.height > MAX_CAPTURE_HEIGHT) {
        fprintf(stderr,
                "Invalid frame dimensions returned by VI\n");
        goto cleanup;
    }

    pixel_count =
        (uint64_t)frame.width *
        (uint64_t)frame.height;

    if (pixel_count > SIZE_MAX) {
        fprintf(stderr, "Frame size is too large\n");
        goto cleanup;
    }

    /*
     * /proc/umap/vi reports sp420.
     *
     * Expected:
     *   Y = width * height
     *   C = width * height / 2
     */
    y_size = (size_t)pixel_count;
    c_size = (size_t)(pixel_count / 2u);

    if (y_size > SIZE_MAX - c_size) {
        fprintf(stderr, "Total frame size overflow\n");
        goto cleanup;
    }

    total_size = y_size + c_size;

    printf("\nExpected SP420 layout:\n");
    printf("Y size     : %zu bytes\n", y_size);
    printf("C size     : %zu bytes\n", c_size);
    printf("Total size : %zu bytes\n", total_size);

    if (plane_distance > 0 &&
        (uint64_t)plane_distance != (uint64_t)y_size) {
        printf("\nWARNING:\n");
        printf("C-Y address distance does not equal width*height.\n");
        printf("The VI buffer may use stride, alignment, or padding.\n");
        printf("The first capture will still use the nominal SP420 sizes.\n");
    }

    if (!valid_mmz_range(frame.phys_addr_y, y_size)) {
        fprintf(stderr,
                "Y plane is outside the known MMZ range\n");
        goto cleanup;
    }

    if (!valid_mmz_range(frame.phys_addr_c, c_size)) {
        fprintf(stderr,
                "C plane is outside the known MMZ range\n");
        goto cleanup;
    }

    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);

    if (mem_fd < 0) {
        fprintf(stderr,
                "Cannot open /dev/mem: %s\n",
                strerror(errno));
        goto cleanup;
    }

    if (map_plane(mem_fd,
                  frame.phys_addr_y,
                  y_size,
                  &y_plane) != 0) {
        goto cleanup;
    }

    if (map_plane(mem_fd,
                  frame.phys_addr_c,
                  c_size,
                  &c_plane) != 0) {
        goto cleanup;
    }

    out_fd = open(output_path,
                  O_WRONLY | O_CREAT | O_TRUNC,
                  0644);

    if (out_fd < 0) {
        fprintf(stderr,
                "Cannot create %s: %s\n",
                output_path,
                strerror(errno));
        goto cleanup;
    }

    printf("\nWriting Y plane...\n");

    if (write_all(out_fd, y_plane.data, y_size) != 0) {
        fprintf(stderr,
                "Failed to write Y plane: %s\n",
                strerror(errno));
        goto cleanup;
    }

    printf("Writing chroma plane...\n");

    if (write_all(out_fd, c_plane.data, c_size) != 0) {
        fprintf(stderr,
                "Failed to write chroma plane: %s\n",
                strerror(errno));
        goto cleanup;
    }

    if (fsync(out_fd) != 0) {
        fprintf(stderr,
                "fsync failed: %s\n",
                strerror(errno));
        goto cleanup;
    }

    printf("\nCapture complete.\n");
    printf("File       : %s\n", output_path);
    printf("Bytes      : %zu\n", total_size);

    exit_status = 0;

cleanup:
    if (out_fd >= 0)
        close(out_fd);

    unmap_plane(&c_plane);
    unmap_plane(&y_plane);

    if (mem_fd >= 0)
        close(mem_fd);

    /*
     * Always return the borrowed VI frame, including error paths.
     */
    if (frame_acquired) {
        if (release_frame(vi_fd, &frame) != 0)
            exit_status = 1;
        else
            printf("VI frame released successfully.\n");
    }

    if (vi_fd >= 0)
        close(vi_fd);

    printf("\nResult: %s\n",
           exit_status == 0 ? "OK" : "FAILED");

    return exit_status;
}
