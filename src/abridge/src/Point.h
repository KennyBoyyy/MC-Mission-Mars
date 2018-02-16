#ifndef POINT_H
#define POINT_H


class Point {
public:
  int left;
  int right;
  int terminal;

  Point(int l, int r, int t) : left(l), right(r), terminal(t){}

  Point(){
      left = 0;
      right = 0;
      terminal = 0;
  }
};

#endif // POINT_H
