#include <iostream>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
using namespace std;

int num=0;
pthread_mutex_t mylock=PTHREAD_MUTEX_INITIALIZER;
void *add(void *arg) {
  int argl = (long)arg;
  int i = 0,tmp;
  for (; i <500; i++)
  {
    pthread_mutex_lock(&mylock);
    tmp=num+1;
    num=tmp;
    cout << num << "  " << argl << endl;
    pthread_mutex_unlock(&mylock);
  }
  return ((void *)0);
}
void *sub(void *arg)
{
  int argl = (long)arg;
  int i=0,tmp;
  for(;i<500;i++)
  {
    pthread_mutex_lock(&mylock);
    tmp=num-1;
    num=tmp;
    cout << num << " " << argl << endl;
    pthread_mutex_unlock(&mylock);
  }
  return ((void *)0);
}
int main(int argc, char** argv) {
   
  pthread_t tid1,tid2;
  int err;
  void *tret;
  int i = 0;
  int j = 1;
  err=pthread_create(&tid1,NULL,add,(void *)i);
  err=pthread_create(&tid2,NULL,sub,(void *)j);
  err=pthread_join(tid1,&tret);
  err=pthread_join(tid2,&tret);
  return 0;
}