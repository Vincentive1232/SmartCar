#include "SEU_Signal_Image_Process.h"


uint8 Bin_Image[Image_width][Image_height] = {0};//用于处理的图像
uint8 Liantong_Image[Image_width][Image_height] = {0};//连通域标记图像（得到连通域标记）
Link label_area[200];//连通域特征信息记录数组（包含上下左右和中心点亮度）


short Threshold = 55;//固定阈值

int center_col = 0;//用于计算的中心列坐标
int center_row = 0;//用于计算的中心行坐标
int center_final_col = 0;//用于传值到Dir_Control()的中心列坐标
int center_final_row = 0;//用于传值到Dir_Control()的中心行坐标
int center_final_col_line = 0;
int sum1 = 0;//全图白点个数总数
int sum_exp = 0;

int textlabel = 0;//连通域数量

int pic_up_Threshold =0;//变切割线行数
int summ = 0;
static int NeighborDirection[8][2] = {{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}};//八邻域搜索数组（上下左右等）

int brake_flag = 0;//刹车标志位
int Link_on_or_off = 1;

/************************************************************************
*  函数名称：PushQueue(Queue *queue, int data)
*  功能说明：入队函数
*  参数说明：Queue *queue -> 队列结构体变量
            int data -> 入队元素
*  函数返回：无
*  修改时间：2022年7月10日
*  备    注：无
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
*  函数名称：PopQueue(Queue *queue)
*  功能说明：出队函数
*  参数说明：Queue *queue -> 队列结构体变量
*  函数返回：int -> 出队元素
*  修改时间：2022年7月10日
*  备    注：无
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
*  函数名称：short GetOSTU (unsigned char tmImage[LCDH][LCDW])
*  功能说明：大津法求阈值大小
*  参数说明：tmImage ： 图像数据
*  函数返回：无
*  修改时间：2011年10月28日
*  备    注：  GetOSTU(Image_Use);//大津法阈值
Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
3) i表示分类的阈值，也即一个灰度级，从0开始迭代 1
4) 通过归一化的直方图，统计0~i 灰度级的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像
的比例w0，        并统计前景像素的平均灰度u0；统计i~255灰度级的像素(假设像素值在此范围的像素叫做背
景像素)  * 所占整幅图像的比例w1，并统计背景像素的平均灰度u1；
5) 计算前景像素和背景像素的方差 g = w0*w1*(u0-u1) (u0-u1)
6) i++；转到4)，直到i为256时结束迭代
7) 将最大g相应的i值作为图像的全局阈值
缺陷:OSTU算法在处理光照不均匀的图像的时候，效果会明显不好，因为利用的是全局像素信息。
************************************************************************/
short GetOSTU(uint8 tmImage[Image_width][Image_height])
{
  pic_up_Threshold=(MAX(eulerAngle.pitch,-30)+30)*2.222;/////////按照pitch来先设定一个上届
  signed short i, j;
  unsigned long Amount = 0;
  unsigned long PixelBack = 0;
  unsigned long PixelshortegralBack = 0;
  unsigned long Pixelshortegral = 0;
  signed long PixelshortegralFore = 0;
  signed long PixelFore = 0;
  float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
  signed short MinValue=255, MaxValue=0;
  unsigned char HistoGram[256]={0};              //
  
  
//  for (j = 0; j < 256; j++)
//    HistoGram[j] = 0; //初始化灰度直方图
  
  for (j = pic_up_Threshold; j < Image_width; j++)
  {
    for (i = 0; i < Image_height; i++)
    {
      //MaxValue=MAX(tmImage[j][i],MaxValue);
      //MinValue=MIN(tmImage[j][i],MinValue);      
      HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
    }
  }
  
  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值
  
  if (MaxValue == MinValue)
    return MaxValue;         // 图像中只有一个颜色
  if (MinValue + 1 == MaxValue)
    return MinValue;        // 图像中只有二个颜色
  
  for (j = MinValue; j <= MaxValue; j++)
    Amount += HistoGram[j];        //  像素总数
  
  
  Pixelshortegral = 0;
  
  
  
  for (j = MinValue; j <= MaxValue; j++)
  {
    Pixelshortegral += HistoGram[j] * j;        //灰度值总数     mg
  }
  
  
  
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)
  {
    PixelBack = PixelBack + HistoGram[j];     //前景像素点数     
   // PixelFore = Amount - PixelBack;           //背景像素点数
    OmegaBack = (float) PixelBack / Amount;   //前景像素百分比   p1
    //OmegaFore = (float) PixelFore / Amount;   //背景像素百分比
    PixelshortegralBack += (float)HistoGram[j] * j;  //前景灰度值     m
    //PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //背景灰度值
   //MicroBack = (float) PixelshortegralBack / PixelBack;   //前景灰度百分比
   //MicroFore = (float) PixelshortegralFore / PixelFore;   //背景灰度百分比
   //Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差
    Sigma=(float)(OmegaBack*Pixelshortegral-PixelshortegralBack)*(OmegaBack*Pixelshortegral-PixelshortegralBack)/(OmegaBack*(1-OmegaBack));
    
    if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
    {
      SigmaB = Sigma;
      if(j>50)Threshold = j;
      
    }
  }
  return Threshold;                        //返回最佳阈值;
}






/************************************************************************
*  函数名称：SearchNeighbor(uint8 *bitmap, int width, int height, uint8 *labelmap, uint8 labelIndex, int pixelIndex, Queue *queue)
*  功能说明：连通域八邻域生长函数
*  参数说明：uint8 *bitmap -> 原始图像
            int width -> 图像宽度（在我们的代码中应为188）
            int height -> 图像长度（在我们的代码中应为120）
            uint8 *labelmap -> 连通域标记图像
            uint8 labelIndex -> 当前连通域标记值
            int pixelIndex -> 当前扫描的八邻域中心点
            Queue *queue -> 使用的队列
*  函数返回：无
*  修改时间：2022年7月10日
*  备    注：无
************************************************************************/
void SearchNeighbor(uint8 *bitmap, int width, int height, uint8 *labelmap, uint8 labelIndex, int pixelIndex, Queue *queue){
  int searchIndex, i, length;/////注意传参的时候必须要数据类型匹配
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
*  函数名称：SearchNeighbor(uint8 *bitmap, int width, int height, uint8 *labelmap, uint8 labelIndex, int pixelIndex, Queue *queue)
*  功能说明：连通域八邻域生长函数
*  参数说明：uint8 *bitmap -> 原始图像
            int width -> 图像宽度（在我们的代码中应为188）
            int height -> 图像长度（在我们的代码中应为120）
            int height_begin -> 开始扫描的行数（与之前切割的行数一样，是为了减少计算量）
            uint8 *labelmap -> 连通域标记图像
*  函数返回：无
*  修改时间：2022年7月10日
*  备    注：无
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
      if(bitmap[index] ==255 && labelmap[index] == 0){//////////這句話出现了问题
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
*  函数名称：Link_Judge(uint8 labelmap[Image_width][Image_height], int textlabel)
*  功能说明：连通域判断函数
*  参数说明：uint8 labelmap[Image_width][Image_height] -> 标记好的连通域图像
            int textlabel -> 连通域个数
*  函数返回：无
*  修改时间：2022年7月10日
*  备    注：无
************************************************************************/
void Link_Judge(uint8 labelmap[Image_width][Image_height], int textlabel){
  //面积条件的计算（实际面积和理论面积）没啥用其实
  float Area_Link = 0;
  float Area_Link_Set = 0;
  
  //灰度条件的计算（实际灰度和理论灰度）也没啥用其实
  float row_set = 0;
  float gray_set = 0;
  
  //无用的连通域个数
  int Useless_Link = 0;
  
  //记录全图最亮连通域中心灰度值
  int brightest = 0;
  int brightest_temp = 0;
  
  //以下四个数据是为了计算倾斜度
  int length1 = 0;
  int length2 = 0;
  int width1 = 0;
  int width2 = 0;
  
  //记录大小面积个数，分界点为60，是为了确定需要去除的噪点大小
  int small_area = 0;
  int big_area = 0;
  
  //判断是否离灯足够近的标志位
  uint8 near_flag = 0;
  center_col = 0;
  center_row = 0;
  
  //
  int flag1 = 0;
  int flag2 = 0;
  
  
  //对连通域数组初始化，防止其保留上一张图残留信息
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
  //对连通域数组初始化，防止其保留上一张图残留信息
  
  
  
  
  //如果没有连通域，直接将最后的中心行列坐标取为0
  if(textlabel == 0){
    center_final_col = 0;
    center_final_row = 0;
  }
  //如果没有连通域，直接将最后的中心行列坐标取为0
  
  else if(textlabel == 1){
    
      //扫描连通域标记图像，将每个连通域的特征（最上最下最左最右等坐标）提取出来
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
      //扫描连通域标记图像，将每个连通域的特征（最上最下最左最右等坐标）提取出来
            
      
      
      //滤除噪点以后进入形状判断，如果形状过于离谱，就将其滤除（滤除方法是将中心坐标设置为0）

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
              //判断其大小是否合适（与位置相比较）
      //        Area_Link = (label_area[i].down - label_area[i].up) * (label_area[i].right - label_area[i].left);
      //        Area_Link_Set = 0.1545*label_area[i].center_r*label_area[i].center_r - 9.908 * label_area[i].center_r + 168.1;//此处待改
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
      //滤除噪点以后进入形状判断，如果形状过于离谱，就将其滤除（滤除方法是将中心坐标设置为0）
  
  
  else{
      //扫描连通域标记图像，将每个连通域的特征（最上最下最左最右等坐标）提取出来
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
      //扫描连通域标记图像，将每个连通域的特征（最上最下最左最右等坐标）提取出来
      
      
      
      
      //计算各个连通域的中心点，并计算出每个连通域的外界四边形的面积，同时判断图像中是否存在大连通域
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
          
          //以下两句判断是否有较大的连通域，即是否开启找寻最亮点的代码，near_flag = 1代表不开启
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
      //计算各个连通域的中心点，并计算出每个连通域的外界四边形的面积，同时判断图像中是否存在大连通域
      
      
      
      
      //如果存在大连通域，则将面积为40个像素点以下的噪点滤除；如果没有大连通域，则将20个像素点以下的噪点滤除
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
      //如果存在大连通域，则将面积为40个像素点以下的噪点滤除；如果没有大连通域，则将20个像素点以下的噪点滤除
      
      
      
      
      
      //滤除噪点以后进入形状判断，如果形状过于离谱，就将其滤除（滤除方法是将中心坐标设置为0）
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
              //判断其大小是否合适（与位置相比较）
      //        Area_Link = (label_area[i].down - label_area[i].up) * (label_area[i].right - label_area[i].left);
      //        Area_Link_Set = 0.1545*label_area[i].center_r*label_area[i].center_r - 9.908 * label_area[i].center_r + 168.1;//此处待改
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
      
      //滤除噪点以后进入形状判断，如果形状过于离谱，就将其滤除（滤除方法是将中心坐标设置为0）
      
      
      
      
      //首先扫描出全图最亮的点的灰度值以及其所属连通域中心坐标；如果此时离灯足够远（near_flag == 0），则启动视最亮点为灯的算法
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
      //首先扫描出全图最亮的点的灰度值以及其所属连通域中心坐标；如果此时离灯足够远（near_flag == 0），则启动视最亮点为灯的算法
      
  }

  
  
  //如果离灯足够近，采取符合要求的连通域中心加和求平均计算最终的中心坐标（带防除零操作）方法；否则直接采取最亮点坐标作为灯的坐标
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
  //如果离灯足够近，采取符合要求的连通域中心加和求平均计算最终的中心坐标（带防除零操作）方法；否则直接采取最亮点坐标作为灯的坐标
}






/************************************************************************
*  函数名称：draw_line()
*  功能说明：信标灯中心十字绘制函数
*  参数说明：void
*  函数返回：无
*  修改时间：2022年7月10日
*  备    注：无
************************************************************************/
void draw_line()////////////////////////画中心线
{
  for(uint8 row = 0; row < 120; row++)
    Bin_Image[row][center_final_col] = 255;
  for(uint8 col = 0; col < 188; col++){
    Bin_Image[center_final_row][col] = 255;
    Bin_Image[pic_up_Threshold][col] = 255;
  }
}





/************************************************************************
*  函数名称：Search_Point(void)
*  功能说明：寻找信标灯
*  参数说明：void
*  函数返回：无
*  修改时间：2022年7月10日
*  备    注：无
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
  
  
  /********此处对图像上半部分进行切除，根据此时的俯仰角来决定切除行数的多寡*******/
  
//  if(eulerAngle.pitch>-46){
//    pic_up_Threshold=(MAX(eulerAngle.pitch,-46)+46)*1.304+5;
//  }
//  else{
//    pic_up_Threshold=(MAX(eulerAngle.pitch,-46)+46)*1.304;
//  }
   pic_up_Threshold=10.5+MIN(eulerAngle.pitch,Angle_Set)*1.16+(MAX(eulerAngle.pitch,Angle_Set)-Angle_Set)*0.5;
  
   // 15+(eulerAngle.pitch+57)*(0.85);/////////按照pitch来先设定一个上届
  
  
  
  if(pic_up_Threshold >= 119){
    pic_up_Threshold = 119;
  }
  else if(pic_up_Threshold <= 0){
    pic_up_Threshold = 0;
  }
  /********切除时应注意不能超过图像总行数*******/
  
  
  
  /********对图像进行二值化处理（固定阈值）*******/
  for (int i = 0; i < Image_width; i++)  
  {
        for (int j = 0; j < Image_height; j++){
            Bin_Image[i][j] = mt9v03x_image[i][j];
            if (Bin_Image[i][j] > Threshold){ //数值越大，显示的内容越多，较浅的图像也能显示出来38
              Bin_Image[i][j] = 255;//白点
              if(i >= pic_up_Threshold){
                sum_exp++;
              }
            }
            else{
              Bin_Image[i][j] = 0;//黑点
            }
        }
   }
  /********对图像进行二值化处理（固定阈值）*******/
  
  
  if(sum_exp < 0){
//      if(Link_on_or_off == 1){
          /********对图像执行连通域扫描（返回值textlabel为连通域个数）*******/
          textlabel = ConnectedComponentLabeling(Bin_Image[0], Image_height, Image_width, pic_up_Threshold, Liantong_Image[0]);
          /********对图像执行连通域扫描（返回值textlabel为连通域个数）*******/
          
          
          
          /********若没有连通域(没有灯）或全图（切割后）连通域个数为1（认为只有灯）*******/
          /********        立刻全图扫描，将所有白点的坐标进行平均得到中心坐标      *******/
          /********       若有多个连通域则进入连通域形状等条件判断（Link_Judge)    *******/
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
//              if (Bin_Image[i][j] > Threshold + 30){ //数值越大，显示的内容越多，较浅的图像也能显示出来38
//                Bin_Image[i][j] = 255;//白点
//              }
//              else{
//                Bin_Image[i][j] = 0;//黑点
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
  /********在显示的图像上绘制中心线*******/
  draw_line();
  /********在显示的图像上绘制中心线*******/
}






















