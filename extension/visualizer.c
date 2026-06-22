#include "pathfinder.h"
#include <stdlib.h>
#include <X11/Xlib.h>
#include "GL/gl.h"
#include "GL/glx.h"
#include <unistd.h>

#include <stdio.h>

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

int main(int argc, char** argv){
    const char* visualfile = argv[1] ? argv[1] : getenv("PATHOUT");
    if (visualfile == NULL){
        fputs("visualizer: expected at least one command line argument or environment variable PATHOUT\n", stderr);
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
        300, 300,
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

    sleep(2);

    //XEvent event;
    //(void)event;

    FILE* in = fopen(visualfile, "rb");
    if (!in){
        fputs("failed to open input file\n", stderr);
        return EXIT_FAILURE;
    }

    size_t width;
    size_t height;
    if (fread(&width, sizeof width, 1, in) != 1){
        fputs("failed to read width\n", stderr);
        return EXIT_FAILURE;
    }
    if (fread(&height, sizeof height, 1, in) != 1) {
        fputs("failed to read height\n", stderr);
        return EXIT_FAILURE;
    }
    uint8_t* costs = malloc(width * height);
    if (!costs){
        fputs("failed to allocate costs array\n", stderr);
        return EXIT_FAILURE;
    }

    GLsizei twidth = ceil2(width);
    GLsizei theight = ceil2(height);


    GLuint tname;
    glGenTextures(1, &tname);
    glBindTexture(GL_TEXTURE_2D, tname);
    glTexParameteri(GL_TEXURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, )

    XCloseDisplay(display);
/*
    depth 1, bits_per_pixel 1, scanline_pad 32
    depth 4, bits_per_pixel 8, scanline_pad 32
    depth 8, bits_per_pixel 8, scanline_pad 32
    depth 15, bits_per_pixel 16, scanline_pad 32
    depth 16, bits_per_pixel 16, scanline_pad 32
    depth 24, bits_per_pixel 32, scanline_pad 32
    depth 32, bits_per_pixel 32, scanline_pad 32
*/
    //XMapWindow
    //XCreateWindow
    //XSetWindowAttributes

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
