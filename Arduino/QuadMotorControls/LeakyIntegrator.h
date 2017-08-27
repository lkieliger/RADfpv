#ifndef LEAKY_INTEGRATOR_H
#define LEAKY_INTEGRATOR_H

class LeakyIntegrator{

    private:
      float currentValue;
      float lambdaFactor;
      
    public:

      LeakyIntegrator(float l): currentValue{0}, lambdaFactor{l}{
        
      }

      void append(float v){
        currentValue = (lambdaFactor * currentValue) + ((1.0 - lambdaFactor) * v);
      }

      float value(){
        return currentValue;
      }
};

#endif
