#ifndef FILTERS_H
#define FILTERS_H

class DSPFilter{
  public:
    virtual void addSample(float sample) = 0;
    virtual float getCurrentValue() = 0;
};

class LeakyIntegrator : public DSPFilter{

    private:
      float currentValue;
      float lambdaFactor;
      
    public:

      LeakyIntegrator(float l): currentValue{0}, lambdaFactor{l}{
        
      }

      void addSample(float v){
        currentValue = (lambdaFactor * currentValue) + ((1.0 - lambdaFactor) * v);
      }

      float getCurrentValue(){
        return currentValue;
      }
};

class MovingAverage : public DSPFilter{
  private:
   int n;
   int head;
   float sum;
   float *vArr;

  public:
    MovingAverage(int s): n{s}, head{0}, sum{0}{
      vArr = new float[s]();
    }

    ~MovingAverage(){
      delete[] vArr;
    }

    void addSample(float value){
      sum -= vArr[head];
      vArr[head] = value / n;
      sum += vArr[head];
      head = (head + 1) % n;
    }

    float getCurrentValue(){
      return  sum;
    }
};

#endif
