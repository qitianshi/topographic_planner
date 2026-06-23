#include "pathfinder.h"
#include <stdlib.h>
#include <X11/Xlib.h>
#include "GL/gl.h"
#include "GL/glx.h"
#include <unistd.h>
#include <inttypes.h>

#include <stdio.h>


#define COLOR_QUEUE (GLubyte[3]){UINT8_MAX, UINT8_MAX, 0}
#define COLOR_TREE (GLubyte[3]){0, UINT8_MAX, 0}
#define COLOR_WHITE (GLubyte[3]){UINT8_MAX, UINT8_MAX, UINT8_MAX}
#define PATTERN_WHOLE 0xffff
#define PATTERN_BROKEN 0xf00f

#define COLOR_COST_LOW (GLubyte[3]){ 0, 0, 81 }
#define COLOR_COST_MID COLOR_WHITE
#define COLOR_COST_HIGH (GLubyte[3]){ 81, 0, 0 }
#define NCOLORCOMP 3

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

GLubyte lerp(GLubyte x, GLubyte y, uint8_t scale, uint8_t max) {
    return (GLubyte)(x + (y - x) * scale / max);
}

void interpol(GLubyte c1[3], GLubyte c2[3], GLubyte res[3], uint8_t scale, uint8_t max) {
    res[0] = lerp(c1[0], c2[0], scale, max);
    res[1] = lerp(c1[1], c2[1], scale, max);
    res[2] = lerp(c1[2], c2[2], scale, max);
}

#define ce check_error(__LINE__);

void specify_vertex(uint32_t pos, uint32_t width){
    printf("vertex: (%d, %d)\n", (GLint)(pos % width), (GLint)(pos / width));
    glVertex2i((GLint)(pos % width), (GLint)(pos / width));
}

void draw_point(uint32_t pos, uint32_t width, GLubyte color[3]) {
    glColor3ubv(color); ce;
    glBegin(GL_POINTS);
    specify_vertex(pos, width);
    glEnd(); ce;
}

void draw_line(uint32_t src, uint32_t dst, uint32_t width, GLushort pattern, GLubyte color[3]){
    glLineStipple(1, pattern); ce;
    glColor3ubv(color); ce;
    glBegin(GL_LINES);
    specify_vertex(src, width);
    specify_vertex(dst, width);
    glEnd(); ce;
}

void draw_tree_line(uint32_t src, uint32_t dst, uint32_t width){
    draw_line(src, dst, width, PATTERN_WHOLE, COLOR_TREE);
}

void draw_queue_line(uint32_t src, uint32_t dst, uint32_t width) {
    draw_line(src, dst, width, PATTERN_BROKEN, COLOR_QUEUE);
}

void erase_line(uint32_t src, uint32_t dst, uint32_t width) {
    glEnable(GL_TEXTURE_2D); ce;
    draw_line(src, dst, width, PATTERN_WHOLE, COLOR_WHITE);
    glDisable(GL_TEXTURE_2D); ce;
}

void draw_queue_point(uint32_t pos, uint32_t width) {
    draw_point(pos, width, COLOR_QUEUE);
}

void draw_tree_point(uint32_t pos, uint32_t width) {
    draw_point(pos, width, COLOR_TREE);
}

int main(int argc, char** argv){
    const char* visualfile = argv[1] ? argv[1] : getenv("PF_VISUALIZE_SEARCH");
    if (visualfile == NULL){
        fputs("visualizer: expected at least one command line argument or environment variable PF_VISUALIZE_SEARCH\n", stderr);
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

    Window win = XCreateWindow(
        display,
        root,
        0, 0,
        1280, 720,
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
    if (fread(&width, sizeof width, 1, in) != 1){
        fputs("failed to read width\n", stderr);
        return EXIT_FAILURE;
    }
    if (fread(&height, sizeof height, 1, in) != 1) {
        fputs("failed to read height\n", stderr);
        return EXIT_FAILURE;
    }
    if (fread(&start, sizeof start, 1, in) != 1) {
        fputs("failed to read start\n", stderr);
        return EXIT_FAILURE;
    }
    if (fread(&dest, sizeof dest, 1, in) != 1) {
        fputs("failed to read dest\n", stderr);
        return EXIT_FAILURE;
    }
    uint8_t* costs = malloc(width * height);
    if (!costs){
        fputs("failed to allocate costs array\n", stderr);
        return EXIT_FAILURE;
    }
    if (fread(costs, sizeof *costs, width * height, in) != width * height){
        fputs("failed to read costs array\n", stderr);
        return EXIT_FAILURE;
    }

    GLsizei twidth = (GLsizei)ceil2(width);
    GLsizei theight = (GLsizei)ceil2(height);
    GLubyte* tdata = calloc((size_t)twidth * (size_t)theight, NCOLORCOMP * sizeof * tdata);

    for (uint32_t y = 0, i = 0, j = 0;
        y < height;
        ++y, j += NCOLORCOMP * ((uint32_t)twidth - width))
    {
        for (uint32_t x = 0; x < width; ++x, ++i, j += NCOLORCOMP) {
            uint8_t cost = costs[i];
            if (cost <= INT8_MAX){
                interpol(COLOR_COST_LOW, COLOR_COST_MID, tdata + j, cost, INT8_MAX);
            } else {
                interpol(COLOR_COST_MID, COLOR_COST_HIGH, tdata + j, cost - INT8_MAX - 1, INT8_MAX);
            }
        }
    }
    free(costs);

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


    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);
    glTexCoord2i(0, (GLint)height);            glVertex2f(-1, 1);
    glTexCoord2i((GLint)width, (GLint)height); glVertex2f(1, 1);
    glTexCoord2i((GLint)width, 0);             glVertex2f(1, -1);
    glTexCoord2i(0, 0);                        glVertex2f(-1, -1);
    glEnd(); ce;
    glFlush(); ce;
    glDisable(GL_TEXTURE_2D); ce;

    glTranslatef(0.5f, 0.5f, 0); ce;
    glOrtho(0, (GLfloat)width, 0, (GLfloat)height, 1, -1); ce;

    //glEnable GL_TEXTURE_GEN_S, GL_TEXTURE_GEN_T, GL_TEXTURE_GEN_R, or GL_TEXTURE_GEN_Q
    //glTexGen
    glMatrixMode(GL_MODELVIEW); ce;
    glTranslatef(0.5f, 0.5f, 0); ce;
    //glTranslatef(0.5f, (GLfloat)height - 1.5f, 0); ce;
    //glScalef(1, -1, 1);

    glMatrixMode(GL_PROJECTION); ce;
    glOrtho(0, (GLfloat)width, 0, (GLfloat)height, 1, -1); ce;

    glEnable(GL_TEXTURE_GEN_S); ce;
    glEnable(GL_TEXTURE_GEN_T); ce;
    //glLineWidth(2); ce;
    //sglPointSize(2); ce;


    for(bool more_updates = true; more_updates;){
        struct pf_visual_update_t update;
        if (fread(&update, sizeof update, 1, in) != 1){
            fputs("failed to read update\n", stderr);
            return EXIT_FAILURE;
        }
        printf("update %d pos=%5"PRIu32", par=%5"PRIu32", old = %5"PRIu32"\n",
            (int)update.type, update.pos, update.parent, update.old_parent);
        switch(update.type){
            case PF_VISUAL_UPDATE_TREE:
                draw_tree_line(update.pos, update.parent, width);
                draw_tree_point(update.pos, width);
                break;
            case PF_VISUAL_UPDATE_QUEUE:
                draw_queue_line(update.pos, update.parent, width);
                draw_queue_point(update.pos, width);
                break;
            case PF_VISUAL_UPDATE_CHANGE_EDGE:
                erase_line(update.pos, update.old_parent, width);
                draw_queue_line(update.pos, update.parent, width);
                break;
            case PF_VISUAL_UPDATE_FINISHED:
                more_updates = false;
                break;
        }
        glFlush();
    }
    getc(stdin);

    XCloseDisplay(display);

    //XNextEvents
    //XMapWindow
    //XCreateWindow

    // XDefaultDepth(
    //XDefaultGC
    //Window XDefaultRootWindow( *display);
    //int XDefaultScreen(Display *display);
    //Screen *XScreenOfDisplay(Display *display, int screen_number);
    //int XDisplayPlanes( *display,  screen_number)
    //unsigned long XNextRequest( *display);
    //int XQLength( *display);
    //int XScreenCount( *display);
    //char *XServerVendor( *display);
    //int XVendorRelease( *display)
    //int XDisplayHeight(Display *display, int screen_number);
    //int XDisplayHeightMM(Display *display, int screen_number);
    //int XWidthOfScreen(Screen * screen);
    //int XHeightOfScreen(Screen * screen);
    //XFree(void* data);
    //XInitThreads
}
