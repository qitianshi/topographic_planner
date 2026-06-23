#include <stdlib.h>
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <inttypes.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include "pathfinder.h"
#include "io.h"
#include "smooth.h"

#define COLOR_QUEUE (GLubyte[3]){UINT8_MAX, UINT8_MAX, 0}
#define COLOR_TREE (GLubyte[3]){0, UINT8_MAX, 0}
#define COLOR_WHITE (GLubyte[3]){UINT8_MAX, UINT8_MAX, UINT8_MAX}
#define PATTERN_WHOLE 0xffff
#define PATTERN_BROKEN 0xf00f
#define COLOR_PATH (GLubyte[3]){UINT8_MAX, 0, UINT8_MAX}
#define COLOR_PATH_FINAL (GLubyte[3]){0, UINT8_MAX, UINT8_MAX}
#define TIME_DELAY 100000

#define COLOR_COST_LOW (GLubyte[3]){ 0, 0, 81 }
#define COLOR_COST_MID COLOR_WHITE
#define COLOR_COST_HIGH (GLubyte[3]){ 81, 0, 0 }
#define NCOLORCOMP 3

/*
#define PROMPT_TWIDTH 64
#define PROMPT_WIDTH 55
#define PROMPT_THEIGHT 8
#define PROMPT_HEIGHT 5
#define A UINT8_MAX

static GLuByte prompt[] = {
A,A,A,A,0,0,A,0,0,A,0,A,A,A,A,A,0,A,A,A,A,0,A,A,A,A,0,0,0,0,0,0,A,A,A,A,0,0,0,0,0,A,0,A,A,0,A,A,A,A,0,A,0,0,A,0,0,0,0,0,0,0,0,0,
A,0,0,0,0,0,A,A,0,A,0,0,0,A,0,0,0,A,0,0,0,0,A,0,0,A,0,0,0,0,0,0,A,0,0,A,0,0,0,0,0,A,A,A,0,0,A,0,0,0,0,A,0,0,A,0,0,0,0,0,0,0,0,0,
A,A,A,A,0,0,A,0,A,A,0,0,0,A,0,0,0,A,A,A,A,0,A,A,A,0,0,0,0,0,0,0,A,A,A,A,0,0,0,0,0,A,A,0,0,0,A,A,A,A,0,A,A,A,A,0,0,0,0,0,0,0,0,0,
A,0,0,0,0,0,A,0,A,A,0,0,0,A,0,0,0,A,0,0,0,0,A,0,0,A,0,0,0,0,0,0,A,0,0,A,0,0,0,0,0,A,A,A,0,0,A,0,0,0,0,0,0,0,A,0,0,0,0,0,0,0,0,0,
A,A,A,A,0,0,A,0,0,A,0,0,0,A,0,0,0,A,A,A,A,0,A,0,0,0,A,0,0,0,0,0,A,0,0,A,0,0,0,0,0,A,0,A,A,0,A,A,A,A,0,A,A,A,A,0,0,0,0,0,0,0,0,0}
#undef A
*/

uint32_t ceil2(uint32_t v){
    --v;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    ++v;
    return v;
}

void check_error(int line){
    while (true){
        GLenum err = glGetError();
        switch(err){
            case GL_NO_ERROR:
                return;
            case GL_INVALID_ENUM:
                fprintf(stderr, "Line %d: GL_INVALID_ENUM\n", line);
                break;
            case GL_INVALID_VALUE:
                fprintf(stderr, "Line %d: GL_INVALID_VALUE\n", line);
                break;
            case GL_INVALID_OPERATION:
                fprintf(stderr, "Line %d: GL_INVALID_OPERATION\n", line);
                break;
            case GL_STACK_OVERFLOW:
                fprintf(stderr, "Line %d: GL_STACK_OVERFLOW\n", line);
                break;
            case GL_STACK_UNDERFLOW:
                fprintf(stderr, "Line %d: GL_STACK_UNDERFLOW\n", line);
                break;
            case GL_OUT_OF_MEMORY:
                fprintf(stderr, "Line %d: GL_OUT_OF_MEMORY\n", line);
                break;
        }
    }
}

static GLubyte lerp(GLubyte x, GLubyte y, uint8_t scale, uint8_t max) {
    return (GLubyte)(x + (y - x) * scale / max);
}

static void interpol(GLubyte c1[3], GLubyte c2[3], GLubyte res[3], uint8_t scale, uint8_t max) {
    res[0] = lerp(c1[0], c2[0], scale, max);
    res[1] = lerp(c1[1], c2[1], scale, max);
    res[2] = lerp(c1[2], c2[2], scale, max);
}

#define ce check_error(__LINE__);

static void specify_vertex(uint32_t pos, uint32_t width){
    glVertex2i((GLint)(pos % width), (GLint)(pos / width));
}

static void draw_point(uint32_t pos, uint32_t width, GLubyte color[3]) {
    glColor3ubv(color); ce;
    glBegin(GL_POINTS);
    specify_vertex(pos, width);
    glEnd(); ce;
}

static void draw_line(uint32_t src, uint32_t dst, uint32_t width, GLushort pattern, GLubyte color[3]){
    glLineStipple(1, pattern); ce;
    glColor3ubv(color); ce;
    glBegin(GL_LINES);
    specify_vertex(src, width);
    specify_vertex(dst, width);
    glEnd(); ce;
}

static void draw_map(uint32_t width, uint32_t height){
    glEnable(GL_TEXTURE_2D);
    glColor3ubv(COLOR_WHITE);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_QUADS);
    glTexCoord2i(0, (GLint)height);            glVertex2f(-0.5f, (GLfloat)height + 0.5f);
    glTexCoord2i((GLint)width, (GLint)height); glVertex2f((GLfloat)width + 0.5f, (GLfloat)height + 0.5f);
    glTexCoord2i((GLint)width, 0);             glVertex2f((GLfloat)width + 0.5f, -0.5f);
    glTexCoord2i(0, 0);                        glVertex2f(-0.5f, -0.5f);
    glEnd(); ce;
    glDisable(GL_TEXTURE_2D); ce;
}

static void draw_path(const struct terrain_map_t* map, const struct dynarr_t* path, GLubyte color[3]){
    glColor3ubv(color);
    glBegin(GL_LINE_STRIP);
    for (size_t i = 0; i < path->length; ++i){
        struct point_2d_t point = *(struct point_2d_t*)dynarr_get(path, i);
        point.x /= map->resolution;
        point.y /= map->resolution;
        glVertex2f(point.x, point.y);
        //draw_line(prev_pos, pos, map->npoints_x, PATTERN_WHOLE, COLOR_PATH);
    }
    glEnd(); ce;
}

static void wait_for_keypress(Display* display, Window win){
    XEvent event;
    for (int i = XPending(display); i; --i)
        XNextEvent(display, &event);
    puts("Press Any Key To Continue");
    XWindowEvent(display, win, KeyPressMask, &event);
}

int main(int argc, char** argv){
    enum io_error_t err;

    uint32_t smooth_steps;
    io_load_smooth_steps(&smooth_steps);

    struct terrain_map_t map;
    if ((err = io_load_map(&map)) != ERR_OK){
        fprintf(stderr, "failed to read map: %s (%d)\n", io_err_to_str(err), (int)err);
        return EXIT_FAILURE;
    }

    uint32_t stepsize;
    if ((err = getenv_uint32("VIS_STEP_SIZE", &stepsize)) != ERR_OK){
        if (err != ERR_VAR_NOT_FOUND){
            fprintf(stderr, "failed to read VIS_STEP_SIZE environment variable: %s (%d)\n", io_err_to_str(err), (int)err);
            return EXIT_FAILURE;
        } else if (argc > 2 && (err = str_to_uint32(argv[2], &stepsize)) != ERR_OK){
            fprintf(stderr, "failed to read stepsize command line argument: %s (%d)", io_err_to_str(err), (int)err);
            return EXIT_FAILURE;
        } else {
            stepsize = 50;
        }
    }
    printf("stepsize = %d\n", (int)stepsize);

    const char* visualfile = argv[1] ? argv[1] : getenv("PF_VISUALIZE_SEARCH");
    if (!visualfile){
        fputs("expected at least one command line argument or environment variable PF_VISUALIZE_SEARCH\n", stderr);
        return EXIT_FAILURE;
    }

    Display* display = XOpenDisplay(NULL);
    if (!display){
        fputs("failed to open display\n", stderr);
        return EXIT_FAILURE;
    }

    int screen = XDefaultScreen(display);


    if(glXQueryExtension(display, NULL, NULL)){
        puts("display supports GLX extension");
    } else {
        fputs("display does not support GLX extension\n", stderr);
        return EXIT_FAILURE;
    }


    int major, minor;
    if (!glXQueryVersion(display, &major, &minor)) {
        fputs("failed to get majrot and minor version of extension\n", stderr);
    }
    else {
        printf("version %d.%d\n", major, minor);
    }
    int attrs[] = {
        GLX_RGBA,
        //GLX_DOUBLEBUFFER,
        GLX_RED_SIZE, 4,
        GLX_GREEN_SIZE, 4,
        GLX_BLUE_SIZE, 4,
        GLX_DEPTH_SIZE, 1,
        None};

    XVisualInfo* vis = glXChooseVisual(display, screen, attrs);
    if (!vis){
        fputs("Undefined GLX attribute encountered in attrs\n", stderr);
    }
    //glXGetConfig(dipplay, vi, GLX_RGBA)

    GLXContext ctx = glXCreateContext(display, vis, NULL, true);
    if (!ctx){
        fputs("failed to create opengl rendering context\n", stderr);
    } else {
        puts("created opengl rendering context");
    }

    XSetWindowAttributes swa;


    Window root = XRootWindow(display, vis->screen);

    swa.colormap = XCreateColormap(
        display,
        root,
        vis->visual,
        AllocNone
    );

    swa.background_pixmap = None;
    swa.border_pixel = 0;
    swa.event_mask = ExposureMask | StructureNotifyMask | KeyPressMask;


    int winheight = XDisplayHeight(display, screen);
    int winwidth = XDisplayWidth(display, screen);
    printf("h=%d, s=%d\n", winheight, winwidth);

    Window win = XCreateWindow(
        display,
        root,
        0, 0,
        (unsigned int)winwidth, (unsigned int)winheight, //1280, 720,
        0,
        vis->depth,
        InputOutput,
        vis->visual,
        CWBackPixmap | CWBorderPixel | CWColormap | CWEventMask,
        &swa
    );

    //XSetWMProperties(display, win, ,)
    if (XSetWMColormapWindows(display, root, (Window[]){root, win}, 2) == 0){
        fputs("failedtointern the WM_COLORMAP_WINDOWS\n", stderr);
    }

    Atom atoms[2] = { XInternAtom(display, "_NET_WM_STATE_FULLSCREEN", False), None };
    XChangeProperty(
        display,
        win,
        XInternAtom(display, "_NET_WM_STATE", False),
        XA_ATOM,
        32,
        PropModeReplace,
        (unsigned char*)atoms,
        1
    );

    XStoreName(display, win, "visualizer");
    XMapWindow(display, win);
    glXMakeCurrent(display, win, ctx);


    //XEvent event;
    //(void)event;

    FILE* in = fopen(visualfile, "rb");
    if (!in){
        fprintf(stderr, "failed to open input file %s\n", visualfile);
        return EXIT_FAILURE;
    }

    uint32_t width, height, start, dest;
    if (fread(&start, sizeof start, 1, in) != 1) {
        fputs("failed to read start\n", stderr);
        return EXIT_FAILURE;
    }
    if (fread(&dest, sizeof dest, 1, in) != 1) {
        fputs("failed to read dest\n", stderr);
        return EXIT_FAILURE;
    }
    width = map.npoints_x;
    height = map.npoints_y;

    GLsizei twidth = (GLsizei)ceil2(width);
    GLsizei theight = (GLsizei)ceil2(height);
    GLubyte* tdata = calloc((size_t)twidth * (size_t)theight, NCOLORCOMP * sizeof * tdata);
    if (!tdata){
        fputs("failed to allocate texture data\n", stderr);
        return EXIT_FAILURE;
    }

    for (uint32_t y = 0, i = 0, j = 0;
        y < height;
        ++y, j += NCOLORCOMP * ((uint32_t)twidth - width))
    {
        for (uint32_t x = 0; x < width; ++x, ++i, j += NCOLORCOMP) {
            uint8_t cost = map.data_cost[i];
            if (cost <= INT8_MAX){
                interpol(COLOR_COST_LOW, COLOR_COST_MID, tdata + j, cost, INT8_MAX);
            } else {
                interpol(COLOR_COST_MID, COLOR_COST_HIGH, tdata + j, cost - INT8_MAX - 1, INT8_MAX);
            }
        }
    }

    GLuint tname;
    glGenTextures(1, &tname); ce;
    glBindTexture(GL_TEXTURE_2D, tname); ce;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); ce;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); ce;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP); ce;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP); ce;
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, twidth, theight, 0, GL_RGB, GL_UNSIGNED_BYTE, tdata); ce;
    free(tdata);

    glMatrixMode(GL_TEXTURE); ce;
    glScalef(1.0f / (GLfloat)twidth, 1.0f / (GLfloat)theight, 1); ce;
    glTranslatef(0.5f, 0.5f, 0); ce;

    glMatrixMode(GL_MODELVIEW); ce;
    glTranslatef(0.5f, 0.5f, 0); ce;

    glMatrixMode(GL_PROJECTION); ce;
    glOrtho(0, (GLfloat)width, 0, (GLfloat)height, 1, -1); ce;

    draw_map(width, height);
    glFlush(); ce;
    //glOrtho(0, (GLfloat)width, 0, (GLfloat)height, 1, -1); ce;

    glEnable(GL_TEXTURE_GEN_S); ce;
    glEnable(GL_TEXTURE_GEN_T); ce;
    glEnable(GL_LINE_STIPPLE);

    uint32_t* parents = calloc(width * height, sizeof *parents);
    if (!parents){
        fputs("failed to allocate parents array\n", stderr);
        return EXIT_FAILURE;
    }

    for(uint32_t more_updates = true, i = 0; more_updates; ++i){
        struct pf_visual_update_t update;
        if (fread(&update, sizeof update, 1, in) != 1){
            fputs("failed to read update\n", stderr);
            return EXIT_FAILURE;
        }
        switch(update.type){
            case PF_VISUAL_UPDATE_TREE:
                draw_line(update.pos, update.parent, width, PATTERN_WHOLE, COLOR_TREE);
                draw_point(update.pos, width, COLOR_TREE);
                break;
            case PF_VISUAL_UPDATE_QUEUE:
                parents[update.pos] = update.parent;
                draw_line(update.pos, update.parent, width, PATTERN_BROKEN, COLOR_QUEUE);
                draw_point(update.pos, width, COLOR_QUEUE);
                break;
            case PF_VISUAL_UPDATE_CHANGE_EDGE:
                parents[update.pos] = update.parent;
                glEnable(GL_TEXTURE_2D); ce;
                draw_line(update.pos, update.old_parent, width, PATTERN_WHOLE, COLOR_WHITE);
                glDisable(GL_TEXTURE_2D); ce;
                draw_line(update.pos, update.parent, width, PATTERN_BROKEN, COLOR_QUEUE);
                break;
            case PF_VISUAL_UPDATE_FINISHED:
                more_updates = false;
                break;
        }
        if (i % stepsize == 0)
            glFlush();
    }
    glFlush();
    glDisable(GL_LINE_STIPPLE);

    struct dynarr_t path;
    pf_create_path(&map, start, dest, parents, &path);

    glLineWidth(2.0f);
    draw_path(&map, &path, COLOR_PATH);
    glFlush(); ce;
    wait_for_keypress(display, win);

    draw_map(width, height);
    draw_path(&map, &path, COLOR_PATH);
    glFlush(); ce;
    usleep(TIME_DELAY);

    for (uint32_t i = 0; i < smooth_steps; ++i){
        glEnable(GL_TEXTURE_2D); ce;
        draw_path(&map, &path, COLOR_WHITE);
        glDisable(GL_TEXTURE_2D); ce;
        smooth_step(&map, &path);
        draw_path(&map, &path, COLOR_PATH);
        glFlush(); ce;
        usleep(TIME_DELAY);
    }
    draw_path(&map, &path, COLOR_PATH_FINAL);
    glFlush(); ce;
    wait_for_keypress(display, win);

    dynarr_free(&path);
    free(parents);
    terrain_map_free(&map);
    fclose(in);
    XFree(vis);
    XCloseDisplay(display);
}
