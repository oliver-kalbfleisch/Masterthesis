#define BOOST_THREAD_PROVIDES_FUTURE
#ifndef MAIN_H
#define MAIN_H
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "trackingCore/cyclicBuffer.h"

#include "trackingCore/UDP_Client.hpp"

#include <opencv2/core/core.hpp>
#include <raspicam/raspicam_cv.h>
#include <unistd.h>
#include <time.h>

#include <boost/asio.hpp>
#include <boost/thread/thread_pool.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <boost/thread/future.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/clamp.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <string>
#endif
