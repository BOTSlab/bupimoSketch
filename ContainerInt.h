#ifndef CONTAINER_INT
#define CONTAINER_INT

class ContainerInt{
  public:

  ContainerInt(int l){
    length = l;  
    listOfValues = new int[length];
    average = 0;
    for(int i = 0; i < length; i++) listOfValues[i] = 0;
  };

  int GetAverage(){
    long averageLong = 0;
    for(int i = 0; i < length; i++) averageLong += listOfValues[i];

    averageLong /= length;

    average = averageLong;

    return average;
  }

  int GetValue(int i){
    if(i < length) return listOfValues[i];
    return -11; 
  }

  void AddValue(int a){
    for(int i = length-1; i >0; i--){
      listOfValues[i] = listOfValues[i-1];
    }

    listOfValues[0] = a;
  }
    
  private:

  int length;
  int *listOfValues;
  int average;
  
};
#endif
