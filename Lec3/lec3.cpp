#include<iostream>
using namespace std;

int main(){
    int x=1;
    cout<<"Value of x = "<<x<<endl;
    cout<<"Adress of x = "<<&x<<endl;

    int *p;
    *p=20;   // not good. because the adress is random. we dont know the name of the value.
    cout<<*p<<endl;
}
