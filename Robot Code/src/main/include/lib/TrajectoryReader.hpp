#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <vector>

using namespace std;

struct TrajectoryPunto
{
    double x, y, time;
};

class TrajectoryReader
{
private:
    std::string fileName;

    fstream trajectoryCSV;
    std::vector<TrajectoryPunto> trajectory;
    std::string line;
    int ctrlPts, lineNum = 0;
    double val;

public:
    TrajectoryReader() = delete;

    TrajectoryReader(string fileName) : fileName(fileName)
    {
    }

    void readTrajectory()
    {
        trajectoryCSV.open(fileName, ios::in);

        // Read data, line by line
        while (std::getline(trajectoryCSV, line))
        {

            // Create a stringstream of the current line
            std::stringstream ss(line);

            // Keep track of the current column index
            int colIdx = 0;

            // Trajectory Point
            TrajectoryPunto point;

            // Grab # of control points from first line
            if (lineNum == 0)
            {
                ss >> ctrlPts;
                lineNum++;
            } // Skip Control Point Lines
            else if (lineNum <= ctrlPts)
            {
                lineNum++;
            }
            else
            {
                // Extract each integer
                while (ss >> val)
                {

                    // Add the current integer to the 'colIdx' column's values vector
                    switch (colIdx)
                    {
                    case 0:
                        point.x = val;
                        break;
                    case 1:
                        point.y = val;
                        break;
                    case 2:
                        point.time = val;
                        break;
                    }

                    // If the next token is a comma, ignore it and move on
                    if (ss.peek() == ',')
                        ss.ignore();

                    // Increment the column index
                    colIdx++;
                }
                trajectory.push_back(point);
            }
        }
    }

    std::vector<TrajectoryPunto> getTrajectory()
    {
        return trajectory;
    }
};
