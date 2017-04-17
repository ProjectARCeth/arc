/* quicksort.c */
#include <iostream>
#include <iterator>
#include <set>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
const int nItems = 5;

int data[] = {4,19,2,1,5};
std::set<int*> test_set;
int* test_array[nItems];

void init_test_set(){
   for(int i=0;i<nItems;++i) test_set.insert(data+i);
   std::set<int*>::iterator iter;
   int index = 0;
   for(iter=test_set.begin(); iter!=test_set.end();++iter){
      test_array[index] = *iter;
      index ++;
   }
}

void print_test_array() {
   std::set<int*>::iterator iter;
   for(iter=test_set.begin(); iter!=test_set.end();++iter){
      std::cout << **iter << " ";
   }
}

void my_qsort(int **links, int **rechts) {
   int **ptr1 = links;
   int **ptr2 = rechts;
   int w, x;
   x = **(links + (rechts - links)/2);
   do {
      while(**ptr1 < x) ptr1++;
      while(**ptr2 > x) ptr2--;
      if(ptr1 > ptr2)
         break;
      w = **ptr1;
      **ptr1 = **ptr2;
      **ptr2 = w;
   } while(++ptr1 <= --ptr2);
   if(links < ptr2)  my_qsort(links, ptr2);
   if(ptr1 < rechts) my_qsort(ptr1, rechts);
}

int main(int argc, char** argv) {
   init_test_set();
   print_test_array();
   std::cout << std::endl;
   my_qsort(test_array, test_array+nItems-1);
   print_test_array();
   std::cout << std::endl;
   return 0;
}