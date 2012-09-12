#include <iostream>
#include <fstream>
#include <libxml/tree.h>
#include <libxml/parser.h>

using namespace std;

xmlDocPtr doc = NULL;
xmlNodePtr root_node = NULL, node = NULL;
ofstream f1, f2, f3, f4, f5, f6, f7, f8, f9;

void addToFile(int, int, int, int, int, int, int, int, int, int);

int main() {

   doc = xmlReadFile("observed_sliders.xml", "UTF-8", 0);
   if (doc != NULL) {
      root_node = xmlDocGetRootElement(doc);
      node = root_node->children;
      while (node->type == XML_TEXT_NODE)
         node = node->next;
   }
   else
      cout<<"XML read error\n";

   f1.open("fish1Size_HD.txt");
   f2.open("fish1X_HD.txt");
   f3.open("fish1Y_HD.txt");
   f4.open("fish2Size_HD.txt");
   f5.open("fish2X_HD.txt");
   f6.open("fish2Y_HD.txt");
   f7.open("activity_HD.txt");
   f8.open("density_HD.txt");
   f9.open("excitement_HD.txt");
  
   xmlChar* xmlString;
   while (node) {
      if (node->type == XML_ELEMENT_NODE) {
         xmlString = xmlGetProp(node, BAD_CAST "id");
         int frameNo = atoi((char*)xmlString);

         xmlNodePtr child1 = node->children;
         while (xmlStrcmp(child1->name, BAD_CAST "FISH1"))
            child1 = child1->next;
         xmlString = xmlGetProp(child1, BAD_CAST "size");
         int size1 = atoi((char*)xmlString);
         xmlString = xmlGetProp(child1, BAD_CAST "x");
         int x1 = atoi((char*)xmlString);
         xmlString = xmlGetProp(child1, BAD_CAST "y");
         int y1 = atoi((char*)xmlString);

         xmlNodePtr child2 = child1->next;
         while (xmlStrcmp(child2->name, BAD_CAST "FISH2"))
            child2 = child2->next;
         xmlString = xmlGetProp(child2, BAD_CAST "size");
         int size2 = atoi((char*)xmlString);
         xmlString = xmlGetProp(child2, BAD_CAST "x");
         int x2 = atoi((char*)xmlString);
         xmlString = xmlGetProp(child2, BAD_CAST "y");
         int y2 = atoi((char*)xmlString);

         xmlNodePtr child3 = child2->next;
         while (xmlStrcmp(child3->name, BAD_CAST "OVERALL"))
            child3 = child3->next;
         xmlString = xmlGetProp(child3, BAD_CAST "activity");
         int activity = atoi((char*)xmlString);
         xmlString = xmlGetProp(child3, BAD_CAST "density");
         int density = atoi((char*)xmlString);
         xmlString = xmlGetProp(child3, BAD_CAST "excitement");
         int excitement = atoi((char*)xmlString);

         cout<<frameNo<<" "<<size1<<" "<<x1<<" "<<y1<<" "<<size2<<" "<<x2<<" "<<y2<<" "<<activity<<" "<<density<<" "<<excitement<<"\n";

         addToFile(frameNo, size1, x1, y1, size2, x2, y2, activity, density, excitement);
      }
      node = node->next;
   }
   xmlFree(xmlString);
   xmlFreeDoc(doc);
   
   f1.close();
   f2.close();
   f3.close();
   f4.close();
   f5.close();
   f6.close();
   f7.close();
   f8.close();
   f9.close();

   return 0;
}

void addToFile(int frameNo, int size1, int x1, int y1, int size2, int x2, int y2, int activity, int density, int excitement) {

   f1<<frameNo<<", "<<size1<<";\n";
   f2<<frameNo<<", "<<x1<<";\n";
   f3<<frameNo<<", "<<y1<<";\n";
   f4<<frameNo<<", "<<size2<<";\n";
   f5<<frameNo<<", "<<x2<<";\n";
   f6<<frameNo<<", "<<y2<<";\n";
   f7<<frameNo<<", "<<activity<<";\n";
   f8<<frameNo<<", "<<density<<";\n";
   f9<<frameNo<<", "<<excitement<<";\n";
}
