#ifndef COLLISIONLOOKUP
#define COLLISIONLOOKUP

#include "dubins.h"
#include "constants.h"
#include <opencv2/opencv.hpp>
namespace HybridAStar {
namespace Lookup {

inline void visulize2DcSpsace(int d,bool * array){ 
cv::Mat img(d, d, CV_8UC1);
for (int i = 0; i < d; ++i) {
    for (int j = 0; j < d; ++j) {
        img.at<uchar>(i, j) = array[i * d + j] ? 255 : 0;
    }
}
cv::namedWindow("Bool Array Visualization", cv::WINDOW_AUTOSIZE);
cv::imshow("Bool Array Visualization", img);
cv::resizeWindow("Bool Array Visualization", 800, 800); // 设置你希望的窗口宽度和高度
cv::waitKey(0);
return;
}
inline void visualize2DSpace(std::vector<std::vector<bool>>& boolArrays, int size) {
    if (boolArrays.size() != 72) {
        std::cerr << "Error: The outer vector must have exactly 72 elements." << std::endl;
        return;
    }

    cv::Mat bigImg(size * 9, size * 8, CV_8UC1);

    for (int i = 0; i < 9; ++i) {
        for (int j = 0; j < 8; ++j) {
            cv::Mat smallImg(size, size, CV_8UC1);
            for (int x = 0; x < size; ++x) {
                for (int y = 0; y < size; ++y) {
                    smallImg.at<uchar>(x, y) = boolArrays[i * 8 + j][x * size + y] ? 255 : 0;
                }
            }
            smallImg.copyTo(bigImg(cv::Rect(j * size, i * size, size, size)));
        }
    }
    cv::namedWindow("Bool Array Visualization", cv::WINDOW_AUTOSIZE);
    cv::imshow("Bool Array Visualization", bigImg);
    cv::resizeWindow("Bool Array Visualization", 500, 500);
    cv::waitKey(0);
}
//###################################################
//                                      DUBINS LOOKUP
//###################################################
inline void dubinsLookup(float* lookup) {
  bool DEBUG = false;
  std::cout << "I am building the Dubin's lookup table...";

  DubinsPath path;

  int width = Constants::dubinsWidth / Constants::cellSize;

  //  // increase the width by one to make it square
  //  if (width % 2 != 0) {
  //    width++;
  //  }

  const int headings = Constants::headings;

  // start and goal vector
  double start[3];
  double goal[] = {0, 0, 0};

  // iterate over the X index of a grid cell
  for (int X = 0; X < width; ++X) {
    start[0] = X;

    // iterate over the Y index of a grid cell
    for (int Y = 0; Y < width; ++Y) {
      start[1] = Y;

      // iterate over the start headings
      for (int h0 = 0; h0 < headings; ++h0) {
        start[2] = Constants::deltaHeadingRad * h0;

        // iterate over the goal headings
        for (int h1 = 0; h1 < headings; ++h1) {
          goal[2] = Constants::deltaHeadingRad * h1;

          // calculate the actual cost
          dubins_init(start, goal, Constants::r, &path);
          lookup[X * headings * headings * width + Y * headings * headings + h0 * headings + h1] = dubins_path_length(&path);

          if (DEBUG && lookup[X * headings * headings * width + Y * headings * headings + h0 * headings + h1] < sqrt(X * X + Y * Y) * 1.000001) {
            std::cout << X << " | " << Y << " | "
                      << Constants::deltaHeadingDeg* h0 << " | "
                      << Constants::deltaHeadingDeg* h1 << " length: "
                      << lookup[X * headings * headings * width + Y * headings * headings + h0 * headings + h1] << "\n";

          }
        }
      }
    }
  }

  std::cout << " done!" << std::endl;
}

//###################################################
//                                   COLLISION LOOKUP
//###################################################

// _____________
// SIGN FUNCTION
inline int sign(double x) {
  if (x >= 0) { return 1; }
  else { return -1; }
}

// _________________________
// COLLISION LOOKUP CREATION
inline void collisionLookup(Constants::config* lookup) {
  bool DEBUG = false;


  std::cout << "I am building the collision lookup table...";
  // cell size
  const float cSize = Constants::cellSize;
  // bounding box size length/width
  const int size = Constants::bbSize;

  struct point {
    double x;
    double y;
  };

  // ______________________
  // VARIABLES FOR ROTATION
  //center of the rectangle
  point c;
  point temp;
  // points of the rectangle
  point p[4];
  point nP[4];

  // turning angle
  double theta;

  // ____________________________
  // VARIABLES FOR GRID TRAVERSAL
  // vector for grid traversal
  point start;
  point end;
  // grid
  bool cSpace[size * size];

  // _____________________________
  // VARIABLES FOR LOOKUP CREATION
  int count = 0;
  const int positionResolution = Constants::positionResolution;
  const int positions = Constants::positions;
  point points[positions];

  // generate all discrete positions within one cell
  for (int i = 0; i < positionResolution; ++i) {
    for (int j = 0; j < positionResolution; ++j) {
      points[positionResolution * i + j].x = 1.f / positionResolution * j + 1/(2.f * positionResolution);
      points[positionResolution * i + j].y = 1.f / positionResolution * i + 1/(2.f * positionResolution);
    }
  }


  for (int q = 0; q < positions; ++q) {
    // set the starting angle to zero;
    theta = Constants::deltaHeadingRad / 2;//不用这么加半度，因为开始算的时候就是0度作为中心计算的;后来发现就是要加半个delta，因为在checkcollision的时候是向下取整

    

    // set points of rectangle
    c.x = (double)size / 2 + points[q].x; // zly:this maybe the center
    c.y = (double)size / 2 + points[q].y;
    if(Constants::useRearAsCenter){
      double frontOffset = Constants::frontHangLength + Constants::wheelBase;
      double backOffset = Constants::rearHangLength;

      // Calculate the half width of the vehicle for left and right offsets
      double halfWidth = Constants::width / 2 / cSize;

      // Front left point
      p[0].x = c.x + frontOffset;
      p[0].y = c.y - halfWidth;

      // Front right point
      p[1].x = c.x + frontOffset;
      p[1].y = c.y + halfWidth;

      // Rear right point
      p[2].x = c.x - backOffset;
      p[2].y = c.y + halfWidth;

      // Rear left point
      p[3].x = c.x - backOffset;
      p[3].y = c.y - halfWidth;
    }else{
      p[0].x = c.x - Constants::length / 2 / cSize;
      p[0].y = c.y - Constants::width / 2 / cSize;

      p[1].x = c.x - Constants::length / 2 / cSize;
      p[1].y = c.y + Constants::width / 2 / cSize;

      p[2].x = c.x + Constants::length / 2 / cSize;
      p[2].y = c.y + Constants::width / 2 / cSize;

      p[3].x = c.x + Constants::length / 2 / cSize;
      p[3].y = c.y - Constants::width / 2 / cSize;
    }

    std::vector<std::vector<bool>> cSpaceRecord;
    for (int o = 0; o < Constants::headings; ++o) {

      if (DEBUG) { std::cout << "\ndegrees: " << theta * 180.f / M_PI << std::endl; }

      // initialize cSpace
      for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
          cSpace[i * size + j] = false;
        }
      }

      // shape rotation
      for (int j = 0; j < 4; ++j) {
        // translate point to origin. p will not change
        temp.x = p[j].x - c.x;
        temp.y = p[j].y - c.y;

        // rotate and shift back
        nP[j].x = temp.x * cos(theta) - temp.y * sin(theta) + c.x; //nP:
        nP[j].y = temp.x * sin(theta) + temp.y * cos(theta) + c.y;
      }

      // create the next angle
      theta += Constants::deltaHeadingRad;

      auto drawLine = [](const point& start, const point& end, bool* cSpace, int size) {
          int X = (int)start.x;
          int Y = (int)start.y;
          cSpace[Y * size + X] = true;

          float lengthOfOneSide = std::sqrt(std::pow(end.x - start.x, 2) + std::pow(end.y - start.y, 2));
          int lengthOfOneSideInt = std::ceil(lengthOfOneSide);

          for (int i = 0; i <= lengthOfOneSideInt; ++i) {
              float x = end.x + i * (start.x - end.x) / (float)lengthOfOneSideInt;
              float y = end.y + i * (start.y - end.y) / (float)lengthOfOneSideInt;
              cSpace[(int)y * size + (int)x] = true;
          }
      };

      // cell traversal clockwise
      for (int k = 0; k < 4; ++k) {
        // create the vectors clockwise
        if (k < 3) {
          start = nP[k];
          end = nP[k + 1];
        } else {
          start = nP[k];
          end = nP[0];
        }
        drawLine(start, end, cSpace, size);
      }
      start = point{(nP[0].x + nP[1].x)/2,(nP[0].y + nP[1].y)/2};
      end = point{(nP[2].x + nP[3].x)/2,(nP[2].y + nP[3].y)/2};
      drawLine(start, end, cSpace, size);
      start = point{(nP[1].x + nP[2].x)/2,(nP[1].y + nP[2].y)/2};
      end = point{(nP[3].x + nP[0].x)/2,(nP[3].y + nP[0].y)/2};
      drawLine(start, end, cSpace, size);
      // GENERATE THE ACTUAL LOOKUP
      count = 0;

      std::vector<bool> cSpaceVector(cSpace, cSpace + size*size);
      cSpaceRecord.push_back(cSpaceVector);

      for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
          if (cSpace[i * size + j]) {
            Constants::relPos rP;
            rP.x = j - (int)c.x;
            rP.y = i - (int)c.y;
            lookup[q * Constants::headings + o].pos.push_back(rP);
            count++;
          }
        }
      }

      lookup[q * Constants::headings + o].length = count;

      if (DEBUG) {
        //DEBUG
        for (int i = 0; i < size; ++i) {
          std::cout << "\n";

          for (int j = 0; j < size; ++j) {
            if (cSpace[i * size + j]) {
              std::cout << "#";
            } else {
              std::cout << ".";
            }
          }
        }

        //TESTING
        std::cout << "\n\nthe center of " << q* Constants::headings + o << " is at " << c.x << " | " << c.y << std::endl;

        for (int i = 0; i < lookup[q * Constants::headings + o].length; ++i) {
          std::cout << "[" << i << "]\t" << lookup[q * Constants::headings + o].pos[i].x << " | " << lookup[q * Constants::headings + o].pos[i].y << std::endl;
        }
      }
    }
    if(DEBUG){
        visualize2DSpace(cSpaceRecord, size);
    }
  }

  std::cout << " done!" << std::endl;

}
}
}
#endif // LOOKUP

