#include "tf/tf.h"
#include "math.h"
using namespace std;
int main()
{
    tf::Quaternion q;
    float final_degree = 3.14/2.0, step = 3.14/4.0;
//    cout<<"[";
//    for(int i=0;i<=int(final_degree/step);i++)
//    {
//        for(int j=0;j<=int(final_degree/step);j++)
//        {
//            for(int k=0;k<=int(final_degree/step);k++)
//            {
//                q.setEuler(i*step, j*step, k*step);
//                cout<<"                                       ";
//                cout<<"["<<q.getX()<<", "<<q.getY()<<", "<<q.getZ()<<", "<<q.getW()<<"],"<<endl;
//            }
//        }
//    }
//    cout<<"]"<<endl;
    cout<<"[";
    for(int i=0;i<=int(final_degree/step);i++)
    {
        q.setEuler(i*step, 0, 0);
        cout<<"                                       ";
        cout<<"["<<q.getX()<<", "<<q.getY()<<", "<<q.getZ()<<", "<<q.getW()<<"],"<<endl;
        if(i==0)continue;
        q.setEuler(0, i*step, 0);
        cout<<"                                       ";
        cout<<"["<<q.getX()<<", "<<q.getY()<<", "<<q.getZ()<<", "<<q.getW()<<"],"<<endl;
        q.setEuler(0, 0, i*step);
        cout<<"                                       ";
        cout<<"["<<q.getX()<<", "<<q.getY()<<", "<<q.getZ()<<", "<<q.getW()<<"],"<<endl;
    }
    cout<<"]"<<endl;
    return 0;
}
