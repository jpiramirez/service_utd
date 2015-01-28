#include "targetstateestimator.h"

int main(int argc, char **argv)
{
    targetStateEstimator tse(10, 10, 1, 0, 0.1, 0.1);
    Rect r;
    r.x = 2;
    r.width = 2;
    r.y = 2;
    r.height = 5;
    tse.updateGrid(r, false);
    Mat grid = tse.getGrid();
    cout << grid;
    r.x = 5;
    r.y = 5;
    tse.updateGrid(r, true);
    cout << grid << endl;
    tse.MLE();
    cout << tse.mean << endl;
    cout << tse.std << endl;
    return 0;
}
