#include "SEU_Signal_Image_Process.h"


uint8 Bin_Image[Image_width][Image_height] = {0};//���ڴ����ͼ��
uint8 Liantong_Image[Image_width][Image_height] = {0};//��ͨ����ͼ�񣨵õ���ͨ���ǣ�
Link label_area[200];//��ͨ��������Ϣ��¼���飨�����������Һ����ĵ����ȣ�


short Threshold = 55;//�̶���ֵ

int center_col = 0;//���ڼ��������������
int center_row = 0;//���ڼ��������������
int center_final_col = 0;//���ڴ�ֵ��Dir_Control()������������
int center_final_row = 0;//���ڴ�ֵ��Dir_Control()������������
int center_final_col_line = 0;
int sum1 = 0;//ȫͼ�׵��������
int sum_exp = 0;

int textlabel = 0;//��ͨ������

int pic_up_Threshold =0;//���и�������
int summ = 0;
static int NeighborDirection[8][2] = {{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}};//�������������飨�������ҵȣ�

int brake_flag = 0;//ɲ����־λ
int Link_on_or_off = 1;

/************************************************************************
*  �������ƣ�PushQueue(Queue *queue, int data)
*  ����˵������Ӻ���
*  ����˵����Queue *queue -> ���нṹ�����
            int data -> ���Ԫ��
*  �������أ���
*  �޸�ʱ�䣺2022��7��10��
*  ��    ע����
************************************************************************/
void PushQueue(Queue *queue, int data){
  QNode *p = NULL; 
  p = (QNode*)malloc(sizeof(QNode));
  p->data = data; 
  if(queue->first == NULL){ 
    queue->first = p;
    queue->last = p; p->next = NULL; 
  }
  else{ 
    p->next = NULL;
    queue->last->next = p; 
    queue->last = p; 
  }
  summ++;
}




/************************************************************************
*  �������ƣ�PopQueue(Queue *queue)
*  ����˵�������Ӻ���
*  ����˵����Queue *queue -> ���нṹ�����
*  �������أ�int -> ����Ԫ��
*  �޸�ʱ�䣺2022��7��10��
*  ��    ע����
************************************************************************/
int PopQueue(Queue *queue){ 
  QNode *p = NULL; 
  int data;
  if(queue->first == NULL){
    return -1; 
  }
  p = queue->first;
  data = p->data;
  if(queue->first->next == NULL){
    queue->first = NULL; queue->last = NULL;
  }
  else{
    queue->first = p->next;
  }
  free(p); 
  summ--;
  return data; 
}






/************************************************************************
*  �������ƣ�short GetOSTU (unsigned char tmImage[LCDH][LCDW])
*  ����˵�����������ֵ��С
*  ����˵����tmImage �� ͼ������
*  �������أ���
*  �޸�ʱ�䣺2011��10��28��
*  ��    ע��  GetOSTU(Image_Use);//�����ֵ
Ostu������������������ͨ��ͳ������ͼ���ֱ��ͼ������ʵ��ȫ����ֵT���Զ�ѡȡ�����㷨����Ϊ��
1) �ȼ���ͼ���ֱ��ͼ������ͼ�����е����ص㰴��0~255��256��bin��ͳ������ÿ��bin�����ص�����
2) ��һ��ֱ��ͼ��Ҳ����ÿ��bin�����ص����������ܵ����ص�
3) i��ʾ�������ֵ��Ҳ��һ���Ҷȼ�����0��ʼ���� 1
4) ͨ����һ����ֱ��ͼ��ͳ��0~i �Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ���ǰ������) ��ռ����ͼ��
�ı���w0��        ��ͳ��ǰ�����ص�ƽ���Ҷ�u0��ͳ��i~255�Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ�����
������)  * ��ռ����ͼ��ı���w1����ͳ�Ʊ������ص�ƽ���Ҷ�u1��
5) ����ǰ�����غͱ������صķ��� g = w0*w1*(u0-u1) (u0-u1)
6) i++��ת��4)��ֱ��iΪ256ʱ��������
7) �����g��Ӧ��iֵ��Ϊͼ���ȫ����ֵ
ȱ��:OSTU�㷨�ڴ�����ղ����ȵ�ͼ���ʱ��Ч�������Բ��ã���Ϊ���õ���ȫ��������Ϣ��
************************************************************************/
short GetOSTU(uint8 tmImage[Image_width][Image_height])
{
  pic_up_Threshold=(MAX(eulerAngle.pitch,-30)+30)*2.222;/////////����pitch�����趨һ���Ͻ�
  signed short i, j;
  unsigned long Amount = 0;
  unsigned long PixelBack = 0;
  unsigned long PixelshortegralBack = 0;
  unsigned long Pixelshortegral = 0;
  signed long PixelshortegralFore = 0;
  signed long PixelFore = 0;
  float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��;
  signed short MinValue=255, MaxValue=0;
  unsigned char HistoGram[256]={0};              //
  
  
//  for (j = 0; j < 256; j++)
//    HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ
  
  for (j = pic_up_Threshold; j < Image_width; j++)
  {
    for (i = 0; i < Image_height; i++)
    {
      //MaxValue=MAX(tmImage[j][i],MaxValue);
      //MinValue=MIN(tmImage[j][i],MinValue);      
      HistoGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
    }
  }
  
  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //��ȡ��С�Ҷȵ�ֵ
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //��ȡ���Ҷȵ�ֵ
  
  if (MaxValue == MinValue)
    return MaxValue;         // ͼ����ֻ��һ����ɫ
  if (MinValue + 1 == MaxValue)
    return MinValue;        // ͼ����ֻ�ж�����ɫ
  
  for (j = MinValue; j <= MaxValue; j++)
    Amount += HistoGram[j];        //  ��������
  
  
  Pixelshortegral = 0;
  
  
  
  for (j = MinValue; j <= MaxValue; j++)
  {
    Pixelshortegral += HistoGram[j] * j;        //�Ҷ�ֵ����     mg
  }
  
  
  
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)
  {
    PixelBack = PixelBack + HistoGram[j];     //ǰ�����ص���     
   // PixelFore = Amount - PixelBack;           //�������ص���
    OmegaBack = (float) PixelBack / Amount;   //ǰ�����ذٷֱ�   p1
    //OmegaFore = (float) PixelFore / Amount;   //�������ذٷֱ�
    PixelshortegralBack += (float)HistoGram[j] * j;  //ǰ���Ҷ�ֵ     m
    //PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //�����Ҷ�ֵ
   //MicroBack = (float) PixelshortegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
   //MicroFore = (float) PixelshortegralFore / PixelFore;   //�����ҶȰٷֱ�
   //Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //������䷽��
    Sigma=(float)(OmegaBack*Pixelshortegral-PixelshortegralBack)*(OmegaBack*Pixelshortegral-PixelshortegralBack)/(OmegaBack*(1-OmegaBack));
    
    if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
    {
      SigmaB = Sigma;
      if(j>50)Threshold = j;
      
    }
  }
  return Threshold;                        //���������ֵ;
}






/************************************************************************
*  �������ƣ�SearchNeighbor(uint8 *bitmap, int width, int height, uint8 *labelmap, uint8 labelIndex, int pixelIndex, Queue *queue)
*  ����˵������ͨ���������������
*  ����˵����uint8 *bitmap -> ԭʼͼ��
            int width -> ͼ���ȣ������ǵĴ�����ӦΪ188��
            int height -> ͼ�񳤶ȣ������ǵĴ�����ӦΪ120��
            uint8 *labelmap -> ��ͨ����ͼ��
            uint8 labelIndex -> ��ǰ��ͨ����ֵ
            int pixelIndex -> ��ǰɨ��İ��������ĵ�
            Queue *queue -> ʹ�õĶ���
*  �������أ���
*  �޸�ʱ�䣺2022��7��10��
*  ��    ע����
************************************************************************/
void SearchNeighbor(uint8 *bitmap, int width, int height, uint8 *labelmap, uint8 labelIndex, int pixelIndex, Queue *queue){
  int searchIndex, i, length;/////ע�⴫�ε�ʱ�����Ҫ��������ƥ��
  labelmap[pixelIndex] = labelIndex;
  length = width * height;
  for(i = 0;i < 8;i++){
    searchIndex = pixelIndex + NeighborDirection[i][0] * width + NeighborDirection[i][1];
    if(searchIndex > 0 && searchIndex < length && bitmap[searchIndex] == 255 && labelmap[searchIndex] == 0){
      labelmap[searchIndex] = labelIndex;
      PushQueue(queue, searchIndex);
    }
  }
}






/************************************************************************
*  �������ƣ�SearchNeighbor(uint8 *bitmap, int width, int height, uint8 *labelmap, uint8 labelIndex, int pixelIndex, Queue *queue)
*  ����˵������ͨ���������������
*  ����˵����uint8 *bitmap -> ԭʼͼ��
            int width -> ͼ���ȣ������ǵĴ�����ӦΪ188��
            int height -> ͼ�񳤶ȣ������ǵĴ�����ӦΪ120��
            int height_begin -> ��ʼɨ�����������֮ǰ�и������һ������Ϊ�˼��ټ�������
            uint8 *labelmap -> ��ͨ����ͼ��
*  �������أ���
*  �޸�ʱ�䣺2022��7��10��
*  ��    ע����
************************************************************************/
int ConnectedComponentLabeling(uint8 *bitmap, int width, int height, int height_begin, uint8 *labelmap){
  int cx, cy, index, popIndex, labelIndex = 0;
  int sum=0;
  Queue *queue = NULL;
  queue = (Queue*)malloc(sizeof(Queue));
  queue->first = NULL; queue->last = NULL;
  int len =sizeof(Liantong_Image);
  memset(labelmap, 0, len);
  for(cy = height_begin + 1; cy < height - 1; cy++){
    for(cx = 1; cx < width - 1; cx++){
      index = cy * width + cx;
      if(bitmap[index] ==255 && labelmap[index] == 0){//////////�@��Ԓ����������
        labelIndex++;
        SearchNeighbor(bitmap, width, height, labelmap, labelIndex, index, queue);
        popIndex = PopQueue(queue);
        while(popIndex > -1){
          if(sum<1200){
                SearchNeighbor(bitmap, width, height, labelmap, labelIndex, popIndex, queue); 
              }
          popIndex = PopQueue(queue);
          sum++;
        }
        if(sum>=1200){free(queue);return -1;}
        sum=0;
        
      }
    } 
  }
  free(queue);
  return labelIndex;
}





/************************************************************************
*  �������ƣ�Link_Judge(uint8 labelmap[Image_width][Image_height], int textlabel)
*  ����˵������ͨ���жϺ���
*  ����˵����uint8 labelmap[Image_width][Image_height] -> ��Ǻõ���ͨ��ͼ��
            int textlabel -> ��ͨ�����
*  �������أ���
*  �޸�ʱ�䣺2022��7��10��
*  ��    ע����
************************************************************************/
void Link_Judge(uint8 labelmap[Image_width][Image_height], int textlabel){
  //��������ļ��㣨ʵ����������������ûɶ����ʵ
  float Area_Link = 0;
  float Area_Link_Set = 0;
  
  //�Ҷ������ļ��㣨ʵ�ʻҶȺ����ۻҶȣ�Ҳûɶ����ʵ
  float row_set = 0;
  float gray_set = 0;
  
  //���õ���ͨ�����
  int Useless_Link = 0;
  
  //��¼ȫͼ������ͨ�����ĻҶ�ֵ
  int brightest = 0;
  int brightest_temp = 0;
  
  //�����ĸ�������Ϊ�˼�����б��
  int length1 = 0;
  int length2 = 0;
  int width1 = 0;
  int width2 = 0;
  
  //��¼��С����������ֽ��Ϊ60����Ϊ��ȷ����Ҫȥ��������С
  int small_area = 0;
  int big_area = 0;
  
  //�ж��Ƿ�����㹻���ı�־λ
  uint8 near_flag = 0;
  center_col = 0;
  center_row = 0;
  
  //
  int flag1 = 0;
  int flag2 = 0;
  
  
  //����ͨ�������ʼ������ֹ�䱣����һ��ͼ������Ϣ
  for(int i = 0;i < 200;i++){
      label_area[i].right = 0;
      label_area[i].left = 187;
      label_area[i].up = 119;
      label_area[i].down = 0;
      label_area[i].right_row = 0;
      label_area[i].left_row = 0;
      label_area[i].up_col = 0;
      label_area[i].down_col = 0;
      label_area[i].center_r = 0;
      label_area[i].center_c = 0;
      label_area[i].center_grew=0;
  }
  //����ͨ�������ʼ������ֹ�䱣����һ��ͼ������Ϣ
  
  
  
  
  //���û����ͨ��ֱ�ӽ�����������������ȡΪ0
  if(textlabel == 0){
    center_final_col = 0;
    center_final_row = 0;
  }
  //���û����ͨ��ֱ�ӽ�����������������ȡΪ0
  
  else if(textlabel == 1){
    
      //ɨ����ͨ����ͼ�񣬽�ÿ����ͨ������������������������ҵ����꣩��ȡ����
      for (uint8 i = Image_width - 1; i > pic_up_Threshold; i--){
            for (uint8 j = 0; j < Image_height; j++){
              if(labelmap[i][j] != 0 && labelmap[i][j] < 200){
                
                label_area[labelmap[i][j]].down = MAX(i,label_area[labelmap[i][j]].down);
                if(label_area[labelmap[i][j]].down == i){
                  label_area[labelmap[i][j]].down_col = j;
                }
                
                label_area[labelmap[i][j]].up = MIN(i,label_area[labelmap[i][j]].up);
                if(label_area[labelmap[i][j]].up == i){
                  label_area[labelmap[i][j]].up_col = j;
                }
                
                label_area[labelmap[i][j]].right = MAX(j,label_area[labelmap[i][j]].right);
                if(label_area[labelmap[i][j]].right == j){
                  label_area[labelmap[i][j]].right_row = i;
                }
                
                label_area[labelmap[i][j]].left = MIN(j,label_area[labelmap[i][j]].left);
                if(label_area[labelmap[i][j]].left == j){
                  label_area[labelmap[i][j]].left_row = i;
                }
              }
            }
        }
      //ɨ����ͨ����ͼ�񣬽�ÿ����ͨ������������������������ҵ����꣩��ȡ����
            
      
      
      //�˳�����Ժ������״�жϣ������״�������ף��ͽ����˳����˳������ǽ�������������Ϊ0��

        for(uint8 i = 1; i < textlabel + 1; i++){
          label_area[i].center_r = (label_area[i].up + label_area[i].down) / 2;
          label_area[i].center_c = (label_area[i].right + label_area[i].left) / 2;
          if(label_area[i].center_c != 0 && label_area[i].center_r != 0){
             int label_area_length = label_area[i].right - label_area[i].left;
             int label_area_width = label_area[i].down - label_area[i].up;
            
            if((label_area_length < (label_area_width - 3)) || ((label_area_length - 3*label_area_width ) > 0)){
              label_area[i].center_r = 0;
              label_area[i].center_c = 0;
              Useless_Link++;
            }
            else{
              Area_Link = (label_area[i].down - label_area[i].up) * (label_area[i].right - label_area[i].left);
              if(Area_Link >= (100 - 50 * brake_flag) && flag1 == 0){
                  brake_flag = 1;
                  flag1 = 1;
              }
              else if(flag1 == 0){
                  brake_flag = 0;
              }
              
//              if(Area_Link >= 100 && flag2 == 0){
//                  Link_on_or_off = 0;
//                  flag2 = 1;
//              }
//              else if(flag2 == 0){
//                  Link_on_or_off = 1;
//              }
              center_final_col = label_area[i].center_c;
              center_final_row = label_area[i].center_r;
              //�ж����С�Ƿ���ʣ���λ����Ƚϣ�
      //        Area_Link = (label_area[i].down - label_area[i].up) * (label_area[i].right - label_area[i].left);
      //        Area_Link_Set = 0.1545*label_area[i].center_r*label_area[i].center_r - 9.908 * label_area[i].center_r + 168.1;//�˴�����
      //        if((Area_Link > (Area_Link_Set*2)) || (Area_Link < (Area_Link_Set*0.2)) || Area_Link<6){
      //          label_area[i].center_r = 0;
      //          label_area[i].center_c = 0;
      //          Useless_Link++;
      //          ///continue;
      //        }
      //        length1 = (label_area[i].right - label_area[i].up_col) * (label_area[i].right - label_area[i].up_col) + (label_area[i].up - label_area[i].right_row)*(label_area[i].up - label_area[i].right_row);
      //        length2 = (label_area[i].left - label_area[i].down_col) * (label_area[i].left - label_area[i].down_col) + (label_area[i].down - label_area[i].left_row)*(label_area[i].down - label_area[i].left_row);
      //        width1 = (label_area[i].left - label_area[i].up_col) * (label_area[i].left - label_area[i].up_col) + (label_area[i].up - label_area[i].left_row)*(label_area[i].up - label_area[i].left_row);
      //        width2 = (label_area[i].right - label_area[i].down_col) * (label_area[i].right - label_area[i].down_col) + (label_area[i].down - label_area[i].right_row)*(label_area[i].down - label_area[i].right_row);
      //        if(((length1 + length2) - (width1 + width2)) > 100 || ((length1 + length2) - (width1 + width2)) < -100){
      //          label_area[i].center_r = 0;
      //          label_area[i].center_c = 0;
      //          Useless_Link++;
      //        }
            }
          }
        }
      }
      //�˳�����Ժ������״�жϣ������״�������ף��ͽ����˳����˳������ǽ�������������Ϊ0��
  
  
  else{
      //ɨ����ͨ����ͼ�񣬽�ÿ����ͨ������������������������ҵ����꣩��ȡ����
      for (uint8 i = Image_width - 1; i > pic_up_Threshold; i--){
            for (uint8 j = 0; j < Image_height; j++){
              if(labelmap[i][j] != 0 && labelmap[i][j] < 200){
                
                label_area[labelmap[i][j]].down = MAX(i,label_area[labelmap[i][j]].down);
                if(label_area[labelmap[i][j]].down == i){
                  label_area[labelmap[i][j]].down_col = j;
                }
                
                label_area[labelmap[i][j]].up = MIN(i,label_area[labelmap[i][j]].up);
                if(label_area[labelmap[i][j]].up == i){
                  label_area[labelmap[i][j]].up_col = j;
                }
                
                label_area[labelmap[i][j]].right = MAX(j,label_area[labelmap[i][j]].right);
                if(label_area[labelmap[i][j]].right == j){
                  label_area[labelmap[i][j]].right_row = i;
                }
                
                label_area[labelmap[i][j]].left = MIN(j,label_area[labelmap[i][j]].left);
                if(label_area[labelmap[i][j]].left == j){
                  label_area[labelmap[i][j]].left_row = i;
                }
              }
            }
        }
      //ɨ����ͨ����ͼ�񣬽�ÿ����ͨ������������������������ҵ����꣩��ȡ����
      
      
      
      
      //���������ͨ������ĵ㣬�������ÿ����ͨ�������ı��ε������ͬʱ�ж�ͼ�����Ƿ���ڴ���ͨ��
      for(uint8 i = 1; i < textlabel + 1; i++){
          label_area[i].center_r = (label_area[i].up + label_area[i].down) / 2;
          label_area[i].center_c = (label_area[i].right + label_area[i].left) / 2;
          Area_Link = (label_area[i].down - label_area[i].up) * (label_area[i].right - label_area[i].left);
          
          if(Area_Link > (100 - 50 * brake_flag) && flag1 == 0){
              brake_flag = 1;
              near_flag = 1;
              big_area++;
              flag1 = 1;
          }
          else if(flag1 == 0){
              brake_flag = 0;
          }
          
          //���������ж��Ƿ��нϴ����ͨ�򣬼��Ƿ�����Ѱ������Ĵ��룬near_flag = 1��������
//          if(Area_Link >= 100 && flag2 == 0){
//              near_flag = 1;
//              big_area++;
//              Link_on_or_off = 0;
//              flag2 = 1;
//          }
//          else if(flag2 == 0){
//              Link_on_or_off = 1;
//          }
      }
      //���������ͨ������ĵ㣬�������ÿ����ͨ�������ı��ε������ͬʱ�ж�ͼ�����Ƿ���ڴ���ͨ��
      
      
      
      
      //������ڴ���ͨ�������Ϊ40�����ص����µ�����˳������û�д���ͨ����20�����ص����µ�����˳�
      if(big_area >= 1){
        for(uint8 i = 1; i < textlabel + 1; i++){
          Area_Link = (label_area[i].down - label_area[i].up) * (label_area[i].right - label_area[i].left);
          if(Area_Link < 40){
              label_area[i].center_r = 0;
              label_area[i].center_c = 0;
              Useless_Link++;
          }
        }
      }
      else{
        for(uint8 i = 1; i < textlabel + 1; i++){
          Area_Link = (label_area[i].down - label_area[i].up) * (label_area[i].right - label_area[i].left);
          if(Area_Link < 2){
              label_area[i].center_r = 0;
              label_area[i].center_c = 0;
              Useless_Link++;
          }
        }
      }
      //������ڴ���ͨ�������Ϊ40�����ص����µ�����˳������û�д���ͨ����20�����ص����µ�����˳�
      
      
      
      
      
      //�˳�����Ժ������״�жϣ������״�������ף��ͽ����˳����˳������ǽ�������������Ϊ0��
        for(uint8 i = 1; i < textlabel + 1; i++){
          if(label_area[i].center_c != 0 && label_area[i].center_r != 0){
             int label_area_length = label_area[i].right - label_area[i].left;
             int label_area_width = label_area[i].down - label_area[i].up;
            
            if((label_area_length < (label_area_width - 3)) || ((label_area_length - 3*label_area_width ) > 0)){
              label_area[i].center_r = 0;
              label_area[i].center_c = 0;
              Useless_Link++;
            }
            else{
              //�ж����С�Ƿ���ʣ���λ����Ƚϣ�
      //        Area_Link = (label_area[i].down - label_area[i].up) * (label_area[i].right - label_area[i].left);
      //        Area_Link_Set = 0.1545*label_area[i].center_r*label_area[i].center_r - 9.908 * label_area[i].center_r + 168.1;//�˴�����
      //        if((Area_Link > (Area_Link_Set*2)) || (Area_Link < (Area_Link_Set*0.2)) || Area_Link<6){
      //          label_area[i].center_r = 0;
      //          label_area[i].center_c = 0;
      //          Useless_Link++;
      //          ///continue;
      //        }
      //        length1 = (label_area[i].right - label_area[i].up_col) * (label_area[i].right - label_area[i].up_col) + (label_area[i].up - label_area[i].right_row)*(label_area[i].up - label_area[i].right_row);
      //        length2 = (label_area[i].left - label_area[i].down_col) * (label_area[i].left - label_area[i].down_col) + (label_area[i].down - label_area[i].left_row)*(label_area[i].down - label_area[i].left_row);
      //        width1 = (label_area[i].left - label_area[i].up_col) * (label_area[i].left - label_area[i].up_col) + (label_area[i].up - label_area[i].left_row)*(label_area[i].up - label_area[i].left_row);
      //        width2 = (label_area[i].right - label_area[i].down_col) * (label_area[i].right - label_area[i].down_col) + (label_area[i].down - label_area[i].right_row)*(label_area[i].down - label_area[i].right_row);
      //        if(((length1 + length2) - (width1 + width2)) > 100 || ((length1 + length2) - (width1 + width2)) < -100){
      //          label_area[i].center_r = 0;
      //          label_area[i].center_c = 0;
      //          Useless_Link++;
      //        }
            }
          }
        }
      
      //�˳�����Ժ������״�жϣ������״�������ף��ͽ����˳����˳������ǽ�������������Ϊ0��
      
      
      
      
      //����ɨ���ȫͼ�����ĵ�ĻҶ�ֵ�Լ���������ͨ���������ꣻ�����ʱ����㹻Զ��near_flag == 0������������������Ϊ�Ƶ��㷨
      for(uint8 i = 1; i < textlabel + 1; i++){
        if(label_area[i].center_c != 0 && label_area[i].center_r != 0){
          label_area[i].center_grew = mt9v03x_image[label_area[i].center_r][label_area[i].center_c];
          for(int j = 0; j < 8; j++){
              label_area[i].center_grew += mt9v03x_image[label_area[i].center_r + NeighborDirection[j][0]][label_area[i].center_c + NeighborDirection[j][1]];
           }
          label_area[i].center_grew = label_area[i].center_grew / 9.0;
          
          if(near_flag == 0){
            brightest_temp = brightest;
            brightest = MAX(brightest,label_area[i].center_grew);
            if(brightest_temp - brightest != 0){
              center_final_col = label_area[i].center_c;
              center_final_row = label_area[i].center_r;
            }
          }
          else{
            center_col += label_area[i].center_c;
            center_row += label_area[i].center_r;
          }
    //      else{
    //        row_set = eulerAngle.pitch * 1.285 + 31.61 + 50;
    //        gray_set = -0.3217*row_set*row_set + 42.62*row_set - 1187;
    //        if((label_area[i].center_grew > 0.7 * gray_set)){//(label_area[i].center_grew < 1.1 * gray_set) && 
    //            center_col += label_area[i].center_c;
    //            center_row += label_area[i].center_r;
    //        }
    //        else{
    //            center_col += 0*label_area[i].center_c;
    //            center_row += 0*label_area[i].center_r;
    //            Useless_Link++;
    //        }
    //      }
         }   
      }
      //����ɨ���ȫͼ�����ĵ�ĻҶ�ֵ�Լ���������ͨ���������ꣻ�����ʱ����㹻Զ��near_flag == 0������������������Ϊ�Ƶ��㷨
      
  }

  
  
  //�������㹻������ȡ����Ҫ�����ͨ�����ļӺ���ƽ���������յ��������꣨�����������������������ֱ�Ӳ�ȡ������������Ϊ�Ƶ�����
  if(near_flag == 1){
    if(textlabel - Useless_Link == 0){
        center_final_col = 0;
        center_final_row = 0;
    }
    else{
      center_final_col = center_col / (textlabel - Useless_Link);
      center_final_row = center_row / (textlabel - Useless_Link);
    }
  }
  //�������㹻������ȡ����Ҫ�����ͨ�����ļӺ���ƽ���������յ��������꣨�����������������������ֱ�Ӳ�ȡ������������Ϊ�Ƶ�����
}






/************************************************************************
*  �������ƣ�draw_line()
*  ����˵�����ű������ʮ�ֻ��ƺ���
*  ����˵����void
*  �������أ���
*  �޸�ʱ�䣺2022��7��10��
*  ��    ע����
************************************************************************/
void draw_line()////////////////////////��������
{
  for(uint8 row = 0; row < 120; row++)
    Bin_Image[row][center_final_col] = 255;
  for(uint8 col = 0; col < 188; col++){
    Bin_Image[center_final_row][col] = 255;
    Bin_Image[pic_up_Threshold][col] = 255;
  }
}





/************************************************************************
*  �������ƣ�Search_Point(void)
*  ����˵����Ѱ���ű��
*  ����˵����void
*  �������أ���
*  �޸�ʱ�䣺2022��7��10��
*  ��    ע����
************************************************************************/
void Search_Point(void){
  int sum=0;
  int average_row_seek=0;
  int average_col_seek=0;
  int area=0;
  int row_average=0;
  int col_average=0;
  float row_regular = 0;
  float grey_regular = 0;
  sum1 = 0;
  sum_exp = 0;
  
  
  /********�˴���ͼ���ϰ벿�ֽ����г������ݴ�ʱ�ĸ������������г������Ķ��*******/
  
//  if(eulerAngle.pitch>-46){
//    pic_up_Threshold=(MAX(eulerAngle.pitch,-46)+46)*1.304+5;
//  }
//  else{
//    pic_up_Threshold=(MAX(eulerAngle.pitch,-46)+46)*1.304;
//  }
   pic_up_Threshold=10.5+MIN(eulerAngle.pitch,Angle_Set)*1.16+(MAX(eulerAngle.pitch,Angle_Set)-Angle_Set)*0.5;
  
   // 15+(eulerAngle.pitch+57)*(0.85);/////////����pitch�����趨һ���Ͻ�
  
  
  
  if(pic_up_Threshold >= 119){
    pic_up_Threshold = 119;
  }
  else if(pic_up_Threshold <= 0){
    pic_up_Threshold = 0;
  }
  /********�г�ʱӦע�ⲻ�ܳ���ͼ��������*******/
  
  
  
  /********��ͼ����ж�ֵ�������̶���ֵ��*******/
  for (int i = 0; i < Image_width; i++)  
  {
        for (int j = 0; j < Image_height; j++){
            Bin_Image[i][j] = mt9v03x_image[i][j];
            if (Bin_Image[i][j] > Threshold){ //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����38
              Bin_Image[i][j] = 255;//�׵�
              if(i >= pic_up_Threshold){
                sum_exp++;
              }
            }
            else{
              Bin_Image[i][j] = 0;//�ڵ�
            }
        }
   }
  /********��ͼ����ж�ֵ�������̶���ֵ��*******/
  
  
  if(sum_exp < 0){
//      if(Link_on_or_off == 1){
          /********��ͼ��ִ����ͨ��ɨ�裨����ֵtextlabelΪ��ͨ�������*******/
          textlabel = ConnectedComponentLabeling(Bin_Image[0], Image_height, Image_width, pic_up_Threshold, Liantong_Image[0]);
          /********��ͼ��ִ����ͨ��ɨ�裨����ֵtextlabelΪ��ͨ�������*******/
          
          
          
          /********��û����ͨ��(û�еƣ���ȫͼ���и����ͨ�����Ϊ1����Ϊֻ�еƣ�*******/
          /********        ����ȫͼɨ�裬�����а׵���������ƽ���õ���������      *******/
          /********       ���ж����ͨ���������ͨ����״�������жϣ�Link_Judge)    *******/
          if(textlabel == -1){
              center_col = 0;
              center_row = 0;
              sum1 = 0;
              for (int i = Image_width - 1; i > pic_up_Threshold; i--){
                for (int j = 0; j < Image_height; j++){
                  if(Bin_Image[i][j]==255){ 
                     center_col += j;
                     center_row += i;
                     sum1++;
                  }
                }
              }
              center_final_col = center_col/sum1;
              center_final_row = center_row/sum1;
          }
          else{
              Link_Judge(Liantong_Image, textlabel);
          }
//      }
//   else{
//      center_col = 0;
//      center_row = 0;
//      sum1 = 0;
//      for (int i = Image_width - 1; i > pic_up_Threshold; i--){
//        for (int j = 0; j < Image_height; j++){
//          if(Bin_Image[i][j]==255){ 
//             center_col += j;
//             center_row += i;
//             sum1++;
//          }
//        }
//      }
//      center_final_col = center_col/sum1;
//      center_final_row = center_row/sum1;
//    }
  }
  else if(sum_exp < 1){
    center_final_row = 0;
    center_final_col = 0;
  }
  else{
    
    
//      for (int i = Image_width - 1; i > pic_up_Threshold; i--)  
//      {
//          for (int j = 0; j < Image_height; j++){
//              Bin_Image[i][j] = mt9v03x_image[i][j];
//              if (Bin_Image[i][j] > Threshold + 30){ //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����38
//                Bin_Image[i][j] = 255;//�׵�
//              }
//              else{
//                Bin_Image[i][j] = 0;//�ڵ�
//              }
//          }
//       }
      Link_on_or_off = 1;
      center_col = 0;
      center_row = 0;
      sum1 = 0;
      for (int i = Image_width - 1; i > pic_up_Threshold; i--){
        for (int j = 0; j < Image_height; j++){
          if(Bin_Image[i][j]==255){ 
             center_col += j;
             center_row += i;
             sum1++;
          }
        }
      }
      center_final_col = center_col/sum1;
      center_final_row = center_row/sum1;
  }
  /*******************************************************************************/
  /*******************************************************************************/
  /*******************************************************************************/
  

  
  if(center_final_col == 0 && center_final_row == 0){
    center_final_row = 119;
    center_final_col = 187;
  }
//  else if(center_final_col > 170 || center_final_col < 17){
//    center_final_row = 187;
//    center_final_col = 187;
//  }
  /********����ʾ��ͼ���ϻ���������*******/
  draw_line();
  /********����ʾ��ͼ���ϻ���������*******/
}






















