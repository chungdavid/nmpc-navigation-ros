#pragma once

#include <vector>

typedef struct {
    double x;
    double y;
} Point;

typedef struct {
    std::vector<Point> points;
    int num_points;
} Path;

class NmpcNavigation
{
public:
    NmpcNavigation();
    ~NmpcNavigation();

    void initGlobalPath(const Path& global_path);
    const Path& getGlobalPath() const;

private:
    Path global_path_;
};