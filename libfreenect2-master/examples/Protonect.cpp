/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

/** @file Protonect.cpp Main application file. */

#include <iostream>
#include <cstdlib>
#include <signal.h>

/// [headers]
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
/// [headers]
#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
#include "viewer.h"
#endif

/* karan adding prototypes of created functions */
void close_connection(libfreenect2::Freenect2Device *dev,  libfreenect2::Registration *registration);
void read_kinect(libfreenect2::SyncMultiFrameListener *listener, libfreenect2::FrameMap &frames, bool &enable_rgb, bool &enable_depth, libfreenect2::Registration *registration, size_t &framecount, bool &protonect_shutdown, Viewer &viewer, bool &viewer_enabled, libfreenect2::Frame &undistorted, libfreenect2::Frame &registered);

/* kinect configuration storage */
struct kinectConfigStruct
{
  libfreenect2::SyncMultiFrameListener *listener;
  libfreenect2::FrameMap *frames;
  bool *enable_rgb;
  bool *enable_depth;
  libfreenect2::Registration *registration;
  size_t *framecount;
  bool *protonect_shutdown;
  Viewer *viewer;
  bool *viewer_enabled;
  libfreenect2::Frame *undistorted;
  libfreenect2::Frame *registered;
  libfreenect2::Freenect2Device *dev;
  size_t *framemax;
};

typedef struct kinectConfigStruct kinectConfigStruct;

bool protonect_shutdown = false; ///< Whether the running application should shut down.

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

bool protonect_paused = false;
libfreenect2::Freenect2Device *devtopause;

//Doing non-trivial things in signal handler is bad. If you want to pause,
//do it in another thread.
//Though libusb operations are generally thread safe, I cannot guarantee
//everything above is thread safe when calling start()/stop() while
//waitForNewFrame().
void sigusr1_handler(int s)
{
  if (devtopause == 0)
    return;
/// [pause]
  if (protonect_paused)
    devtopause->start();
  else
    devtopause->stop();
  protonect_paused = !protonect_paused;
/// [pause]
}

//The following demostrates how to create a custom logger
/// [logger]
#include <fstream>
#include <cstdlib>
class MyFileLogger: public libfreenect2::Logger
{
private:
  std::ofstream logfile_;
public:
  MyFileLogger(const char *filename)
  {
    if (filename)
      logfile_.open(filename);
    level_ = Debug;
  }
  bool good()
  {
    return logfile_.is_open() && logfile_.good();
  }
  virtual void log(Level level, const std::string &message)
  {
    logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
  }
};
/// [logger]

/// [main]
/**
 * Main application entry point.
 *
 * Accepted argumemnts:
 * - cpu Perform depth processing with the CPU.
 * - gl  Perform depth processing with OpenGL.
 * - cl  Perform depth processing with OpenCL.
 * - <number> Serial number of the device to open.
 * - -noviewer Disable viewer window.
 */
kinectConfigStruct* start_connection(int argc, char *argv[])
/// [main]
{
  std::string program_path(argv[0]);
  std::cerr << "Version: " << LIBFREENECT2_VERSION << std::endl;
  std::cerr << "Environment variables: LOGFILE=<protonect.log>" << std::endl;
  std::cerr << "Usage: " << program_path << " [-gpu=<id>] [gl | cl | cuda | cpu] [<device serial>]" << std::endl;
  std::cerr << "        [-noviewer] [-norgb | -nodepth] [-help] [-version]" << std::endl;
  std::cerr << "        [-frames <number of frames to process>]" << std::endl;
  std::cerr << "To pause and unpause: pkill -USR1 Protonect" << std::endl;
  size_t executable_name_idx = program_path.rfind("Protonect");

  std::string binpath = "/";

  if(executable_name_idx != std::string::npos)
  {
    binpath = program_path.substr(0, executable_name_idx);
  }

  /* --------------------------- karan marking end of initialization ---------------------- */

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
  // avoid flooing the very slow Windows console with debug messages
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Info));
#else
  // create a console logger with debug level (default is console logger with info level)
/// [logging]
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
/// [logging]
#endif
/// [file logging]
  MyFileLogger *filelogger = new MyFileLogger(getenv("LOGFILE"));
  if (filelogger->good())
    libfreenect2::setGlobalLogger(filelogger);
  else
    delete filelogger;
  /// [file logging]

/// [context]
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;
/// [context]

  std::string serial = "";

  bool viewer_enabled = true;
  bool enable_rgb = true;
  bool enable_depth = true;
  int deviceId = -1;
  size_t framemax = -1;

  for(int argI = 1; argI < argc; ++argI)
  {
    const std::string arg(argv[argI]);

    if(arg == "-help" || arg == "--help" || arg == "-h" || arg == "-v" || arg == "--version" || arg == "-version")
    {
      // Just let the initial lines display at the beginning of main
      return 0;
    }
    else if(arg.find("-gpu=") == 0)
    {
      if (pipeline)
      {
        std::cerr << "-gpu must be specified before pipeline argument" << std::endl;
        //return -1;
	return NULL;
      }
      deviceId = atoi(argv[argI] + 5);
    }
    else if(arg == "cpu")
    {
      if(!pipeline)
/// [pipeline]
        pipeline = new libfreenect2::CpuPacketPipeline();
/// [pipeline]
    }
    else if(arg == "gl")
    {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
      std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg == "cl")
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::OpenCLPacketPipeline(deviceId);
#else
      std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg == "cuda")
    {
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::CudaPacketPipeline(deviceId);
#else
      std::cout << "CUDA pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg.find_first_not_of("0123456789") == std::string::npos) //check if parameter could be a serial number
    {
      serial = arg;
    }
    else if(arg == "-noviewer" || arg == "--noviewer")
    {
      viewer_enabled = false;
    }
    else if(arg == "-norgb" || arg == "--norgb")
    {
      enable_rgb = false;
    }
    else if(arg == "-nodepth" || arg == "--nodepth")
    {
      enable_depth = false;
    }
    else if(arg == "-frames")
    {
      ++argI;
      framemax = strtol(argv[argI], NULL, 0);
      if (framemax == 0) {
        std::cerr << "invalid frame count '" << argv[argI] << "'" << std::endl;
        //return -1;
	return NULL;
      }
    }
    else
    {
      std::cout << "Unknown argument: " << arg << std::endl;
    }
  }

  if (!enable_rgb && !enable_depth)
  {
    std::cerr << "Disabling both streams is not allowed!" << std::endl;
    //return -1;
    return NULL;
  }

/// [discovery]
  if(freenect2.enumerateDevices() == 0)
  {
    std::cout << "no device connected!" << std::endl;
    //return -1;
    return NULL;
  }

  if (serial == "")
  {
    serial = freenect2.getDefaultDeviceSerialNumber();
  }
/// [discovery]

  if(pipeline)
  {
/// [open]
    dev = freenect2.openDevice(serial, pipeline);
/// [open]
  }
  else
  {
    dev = freenect2.openDevice(serial);
  }

  if(dev == 0)
  {
    std::cout << "failure opening device!" << std::endl;
    //return -1;
    return NULL;
  }

  devtopause = dev;

  signal(SIGINT,sigint_handler);
#ifdef SIGUSR1
  signal(SIGUSR1, sigusr1_handler);
#endif
  protonect_shutdown = false;

/// [listeners]
  int types = 0;
  if (enable_rgb)
    types |= libfreenect2::Frame::Color;
  if (enable_depth)
    types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
  libfreenect2::SyncMultiFrameListener listener(types);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
/// [listeners]

/// [start]
  if (enable_rgb && enable_depth)
  {
    if (!dev->start())
      //return -1;
      return NULL;
  }
  else
  {
    if (!dev->startStreams(enable_rgb, enable_depth))
      //return -1;
      return NULL;
  }

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
/// [start]

/// [registration setup]
  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
/// [registration setup]

  size_t framecount = 0;
#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
  Viewer viewer;
  if (viewer_enabled)
    viewer.initialize();
#else
  viewer_enabled = false;
#endif

  /* --------------------------- karan marking end of initialization ---------------------- */

  /* print everything */
  /*
  std::cout<<"\n printing in initialization";
  std::cout<<"\n"<<(&listener);
  std::cout<<"\n"<<(&frames);
  std::cout<<"\n"<<(&enable_rgb);
  std::cout<<"\n"<<(&enable_depth);
  std::cout<<"\n"<<(registration);
  std::cout<<"\n"<<(&framecount);
  std::cout<<"\n"<<(&protonect_shutdown);
  std::cout<<"\n"<<(&viewer);
  std::cout<<"\n"<<(&viewer_enabled);
  std::cout<<"\n"<<(&undistorted);
  std::cout<<"\n"<<(&registered);
  std::cout<<"\n"<<(dev);
  std::cout<<"\n"<<(&framemax);
  */

  /* assign configuration values */
  kinectConfigStruct kinectConfiguration;
  kinectConfiguration.listener = &listener;
  kinectConfiguration.frames = &frames;
  kinectConfiguration.enable_rgb = &enable_rgb;
  kinectConfiguration.enable_depth = &enable_depth;
  kinectConfiguration.registration = registration;
  kinectConfiguration.framecount = &framecount;
  kinectConfiguration.protonect_shutdown = &protonect_shutdown;
  kinectConfiguration.viewer = &viewer;
  kinectConfiguration.viewer_enabled = &viewer_enabled;
  kinectConfiguration.undistorted = &undistorted;
  kinectConfiguration.registered = &registered;
  kinectConfiguration.dev = dev;
  kinectConfiguration.framemax = &framemax;

  std::cout<<"\n configuration variable address in function: "<<&kinectConfiguration;

  return &kinectConfiguration;
}

void close_connection(libfreenect2::Freenect2Device *dev,  libfreenect2::Registration *registration)
{
  // TODO: restarting ir stream doesn't work!
  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
/// [stop]
  dev->stop();
  dev->close();
/// [stop]

  delete registration;
}

void read_kinect(libfreenect2::SyncMultiFrameListener *listener, libfreenect2::FrameMap &frames, bool &enable_rgb, bool &enable_depth, libfreenect2::Registration *registration, size_t &framecount, bool &protonect_shutdown, Viewer *viewer, bool &viewer_enabled, libfreenect2::Frame *undistorted, libfreenect2::Frame *registered)
{
    listener->waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
/// [loop start]

    if (enable_rgb && enable_depth)
    {
/// [registration]
      registration->apply(rgb, depth, undistorted, registered);
/// [registration]
    }

    framecount++;
    if (!viewer_enabled)
    {
      if (framecount % 100 == 0)
        std::cout << "The viewer is turned off. Received " << framecount << " frames. Ctrl-C to stop." << std::endl;
      listener->release(frames);
      //continue;
      return;
    }

#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
    if (enable_rgb)
    {
      viewer->addFrame("RGB", rgb);
    }
    if (enable_depth)
    {
      viewer->addFrame("ir", ir);
      viewer->addFrame("depth", depth);
    }
    if (enable_rgb && enable_depth)
    {
      viewer->addFrame("registered", registered);
    }

    protonect_shutdown = protonect_shutdown || viewer->render();
#endif

/// [loop end]
    listener->release(frames);
    /** libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100)); */
}

int main(int argc, char *argv[])
{
  /* initialize connection */
  kinectConfigStruct *pKinectConfiguration = start_connection(argc, argv);

  std::cout<<"\n configuration variable address in main: "<<pKinectConfiguration;

  /* print everything */
  /*
  std::cout<<"\n printing in main";
  std::cout<<"\n"<<pKinectConfiguration->listener;
  std::cout<<"\n"<<pKinectConfiguration->frames;
  std::cout<<"\n"<<pKinectConfiguration->enable_rgb;
  std::cout<<"\n"<<pKinectConfiguration->enable_depth;
  std::cout<<"\n"<<pKinectConfiguration->registration;
  std::cout<<"\n"<<pKinectConfiguration->framecount;
  std::cout<<"\n"<<pKinectConfiguration->protonect_shutdown;
  std::cout<<"\n"<<pKinectConfiguration->viewer;
  std::cout<<"\n"<<pKinectConfiguration->viewer_enabled;
  std::cout<<"\n"<<pKinectConfiguration->undistorted;
  std::cout<<"\n"<<pKinectConfiguration->registered;
  std::cout<<"\n"<<pKinectConfiguration->dev;
  std::cout<<"\n"<<pKinectConfiguration->framemax;
  */

  libfreenect2::SyncMultiFrameListener *listener = pKinectConfiguration->listener;
  //libfreenect2::FrameMap frames = *(pKinectConfiguration->frames);
  libfreenect2::FrameMap frames;
  bool enable_rgb = *(pKinectConfiguration->enable_rgb);
  bool enable_depth = *(pKinectConfiguration->enable_depth);
  libfreenect2::Registration *registration = (pKinectConfiguration->registration);
  size_t framecount = *(pKinectConfiguration->framecount);
  bool protonect_shutdown = *(pKinectConfiguration->protonect_shutdown);
  Viewer *viewer = pKinectConfiguration->viewer;
  bool viewer_enabled = *(pKinectConfiguration->viewer_enabled);
  libfreenect2::Frame *undistorted = pKinectConfiguration->undistorted;
  libfreenect2::Frame *registered = pKinectConfiguration->registered;
  libfreenect2::Freenect2Device *dev = (pKinectConfiguration->dev);
  size_t framemax = *(pKinectConfiguration->framemax);
  
  return 0;

#if 0
  /* read data */
  while(!protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax))
  {
    read_kinect(listener, frames, enable_rgb, enable_depth, registration, framecount, protonect_shutdown, viewer, viewer_enabled, undistorted, registered);
  }
  /* close connection */
  close_connection(dev, registration);
  /* default return value */
  return 0;
#endif
}
