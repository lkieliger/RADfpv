#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

class MovingAverage{
  private:
   int n;
   int head;
   float sum;
   float *vArr;

  public:
    MovingAverage(int s): n{s}, sum{0}, head{0}{
      vArr = new float[s]();
    }

    ~MovingAverage(){
      delete[] vArr;
    }

    void append(float value){
      sum -= vArr[head];
      vArr[head] = value / n;
      sum += vArr[head];
      head = (head + 1) % n;
    }

    float value(){
      return  sum;
    }
};

#endif
