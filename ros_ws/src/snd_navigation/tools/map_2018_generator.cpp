#include <cstdio>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

int main()
{
  std::puts("Generate a map for the 2018 Eurobot field");

  const int fieldWidthMm = 3000;
  const int fieldHeightMm = 2000;
  const int resolutionMm = 5;

  std::printf("Map with a resolution of %d mm\n", resolutionMm);

  // Add borders
  const int borderWidth = 1;
  const int fieldWidth = fieldWidthMm / resolutionMm + 2 * borderWidth;
  const int fieldHeight = fieldHeightMm / resolutionMm + 2 * borderWidth;

  cv::Mat field(fieldHeight, fieldWidth, CV_8UC1, cv::Scalar(254));

  cv::rectangle(field, cv::Rect(borderWidth / 2, borderWidth / 2,
                                field.cols - 2 * (borderWidth / 2),
                                field.rows - 2 * (borderWidth / 2)),
                cv::Scalar(0), borderWidth);

  // Add the station
  const int stationWidthMm = 1200;
  const int stationHeightMm = 250;
  const int stationWidth = stationWidthMm / resolutionMm;
  const int stationHeight = stationHeightMm / resolutionMm;
  const int stationX = fieldWidth / 2 - stationWidth / 2;
  const int stationY = fieldHeight - stationHeight - borderWidth;
  cv::rectangle(field, cv::Rect(stationX, stationY, stationWidth, stationHeight),
                cv::Scalar(100), CV_FILLED);

  // Save the map
  const std::string mapBaseFilename = "map_2018";
  const std::string mapImageFilename = mapBaseFilename + ".pgm";
  const std::string mapYamlFilename = mapBaseFilename + ".yaml";
  cv::imwrite(mapImageFilename, field);
  cv::imshow("field", field);
  cv::waitKey(-1);

  const double originX = 0.0;
  const double originY = 0.0;
  const double originYaw = 0.0;

  // Save the YAML map file
  std::ofstream yamlFile(mapYamlFilename);
  yamlFile << "image: " << mapImageFilename << "\nresolution: " << resolutionMm * 1e-3
           << "\norigin: [" << originX << ", " << originY << ", " << originYaw
           << "]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n";

  std::printf("Map generated with the name %s\n", mapImageFilename.c_str());

  return 0;
}
