#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
//#include <libxml/xmlmemory.h>
#include <libxml/tree.h>

using namespace std;

xmlDocPtr doc = NULL;
xmlNodePtr root_node = NULL;
char buff[256];

void addToXml(int, int, int, int, int, int, int, int, int, int);

int main() {

   doc = xmlNewDoc(BAD_CAST "1.0");
   root_node = xmlNewNode(NULL, BAD_CAST "SLIDERS");
   xmlDocSetRootElement(doc, root_node);

   string f1, f2, f3, f4, f5, f6, f7, f8, f9;
   ifstream file1("fish1Size_HD.txt");
   ifstream file2("fish1X_HD.txt");
   ifstream file3("fish1Y_HD.txt");
   ifstream file4("fish2Size_HD.txt");
   ifstream file5("fish2X_HD.txt");
   ifstream file6("fish2Y_HD.txt");
   ifstream file7("activity_HD.txt");
   ifstream file8("density_HD.txt");
   ifstream file9("excitement_HD.txt");

   
   if (file1.is_open() && file2.is_open() && file3.is_open() && file4.is_open() && file5.is_open() && file6.is_open() && file7.is_open() && file8.is_open() && file9.is_open()) {
      while (file1.good()) {
         getline (file1, f1);
         getline (file2, f2);
         getline (file3, f3);
         getline (file4, f4);
         getline (file5, f5);
         getline (file6, f6);
         getline (file7, f7);
         getline (file8, f8);
         getline (file9, f9);


         int frameNo = atoi(f1.substr(0, f1.find(',')).c_str());

         int size1 = atoi(f1.substr(f1.find(' ') + 1, f1.find(';')).c_str()); 
         int x1 = atoi(f2.substr(f2.find(' ') + 1, f2.find(';')).c_str());
         int y1 = atoi(f3.substr(f3.find(' ') + 1, f3.find(';')).c_str()); 
         int size2 = atoi(f4.substr(f4.find(' ') + 1, f4.find(';')).c_str()); 
         int x2 = atoi(f5.substr(f5.find(' ') + 1, f5.find(';')).c_str()); 
         int y2 = atoi(f6.substr(f6.find(' ') + 1, f6.find(';')).c_str()); 
         
         int activity =  atoi(f7.substr(f7.find(' ') + 1, f7.find(';')).c_str()); 
         int density =  atoi(f8.substr(f8.find(' ') + 1, f8.find(';')).c_str()); 
         int excitement = atoi(f9.substr(f9.find(' ') + 1, f9.find(';')).c_str()); 
         cout<<frameNo<<" "<<size1<<" "<<x1<<" "<<y1<<" "<<size2<<" "<<x2<<" "<<y2<<activity<<" "<<density<<" "<<excitement<<"\n";

         addToXml(frameNo, size1, x1, y1, size2, x2, y2, activity, density, excitement);
      }
      file1.close();
      file2.close();
      file3.close();
      file4.close();
      file5.close();
      file6.close();
      file7.close();
      file8.close();
      file9.close();
   }
   else
      cout<<"unable to open";
   xmlSaveFormatFileEnc("sliders.xml", doc, "UTF-8", 1);
   xmlFreeDoc(doc);
   return 0;
}

void addToXml(int frameNo, int size1, int x1, int y1, int size2, int x2, int y2, int activity, int density, int excitement) {

   xmlNodePtr child = xmlNewChild(root_node, NULL, BAD_CAST "FRAME", NULL);
   sprintf(buff, "%d", frameNo);
   xmlNewProp(child, BAD_CAST "id", BAD_CAST (buff));

   xmlNodePtr child1 = xmlNewChild(child, NULL, BAD_CAST "FISH1", NULL);
   sprintf(buff, "%d", size1);
   xmlNewProp(child1, BAD_CAST "size", BAD_CAST (buff));
   sprintf(buff, "%d", x1);
   xmlNewProp(child1, BAD_CAST "x", BAD_CAST (buff));
   sprintf(buff, "%d", y1);
   xmlNewProp(child1, BAD_CAST "y", BAD_CAST (buff));

   xmlNodePtr child2 = xmlNewChild(child, NULL, BAD_CAST "FISH2", NULL);
   sprintf(buff, "%d", size2);
   xmlNewProp(child2, BAD_CAST "size", BAD_CAST (buff));
   sprintf(buff, "%d", x2);
   xmlNewProp(child2, BAD_CAST "x", BAD_CAST (buff));
   sprintf(buff, "%d", y2);
   xmlNewProp(child2, BAD_CAST "y", BAD_CAST (buff));

   xmlNodePtr child3 = xmlNewChild(child, NULL, BAD_CAST "OVERALL", NULL);
   sprintf(buff, "%d", activity);
   xmlNewProp(child3, BAD_CAST "activity", BAD_CAST (buff));
   sprintf(buff, "%d", density);
   xmlNewProp(child3, BAD_CAST "density", BAD_CAST (buff));
   sprintf(buff, "%d", excitement);
   xmlNewProp(child3, BAD_CAST "excitement", BAD_CAST (buff));
}
