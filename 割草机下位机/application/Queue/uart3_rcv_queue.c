#include "uart3_rcv_queue.h"
/*--------------------------------------------------------------------------*/
static void Queue_init(void);
static ST_moving_cmd_queue_p Creat_Moving_Cmd_Node(uint8_t package_id,uint8_t moving_cmd,int16_t v,float w,float s_or_angle);
static void Insert_To_Moving_Cmd_List(ST_moving_cmd_queue_p H);
static uint8_t Get_Node_From_Moving_List_Head(uint8_t *moving_cmd_p,int16_t *v_p,float *w_p,float *s_or_angle_p);
static void Free_Moving_Cmd_List(void);
static void Free_All_Head_Node(void);
static uint8_t List_Is_Null(void);
static uint8_t Get_Last_Package_Id(void);
static uint8_t Get_queue_head_Cmd_Type(void);
static void Update_Remote_Cmd(uint8_t moving_cmd,int16_t v_p,float w_p);
static uint8_t Get_Remote_Cmd(uint8_t *moving_cmd_p,int16_t *v_p,float *w_p);
 /*更新轨迹跟踪命令，只有一个节点的队列*/
static void Update_Track_Tracking_Cmd(uint8_t moving_cmd,uint8_t id,int16_t v_p,float w_p,float s_or_angle_p);
/*从队列链表头部取走数据*/
static uint8_t Get_Track_Tracking_Cmd(uint8_t *moving_cmd_p,int16_t *v_p,float *w_p,float *s_or_angle_p);
ST_moving_cmd_queue_p moving_cmd_queue_head_p = NULL;
/*---------------------------------------------------------------------------*/
 ST_queue_handle queue_handle ={
		Queue_init,
		Creat_Moving_Cmd_Node,
		Insert_To_Moving_Cmd_List,
		Get_Node_From_Moving_List_Head,
		Free_Moving_Cmd_List,
		Free_All_Head_Node,
	  List_Is_Null,
	  Get_Last_Package_Id,
	  Get_queue_head_Cmd_Type,
	  Update_Remote_Cmd,
	  Get_Remote_Cmd,
	  Update_Track_Tracking_Cmd,
	  Get_Track_Tracking_Cmd
};
 ST_queue_handle_p queue_handle_p  = &queue_handle;
/*--------------------------------------------------------------------------*/
/*初始化-实例化头节点-------------------------------------------------------*/
static void Queue_init(void)
{
  moving_cmd_queue_head_p = (ST_moving_cmd_queue_p)malloc(sizeof(ST_moving_cmd_queue));
	moving_cmd_queue_head_p->moving_cmd = 1;
	moving_cmd_queue_head_p->v = 2;
	moving_cmd_queue_head_p->w = 3;
	moving_cmd_queue_head_p->s_or_angle = 4;
	moving_cmd_queue_head_p->next = NULL;
}
/*节点创建*/
static ST_moving_cmd_queue_p Creat_Moving_Cmd_Node(uint8_t package_id,uint8_t moving_cmd,int16_t v,float w,float s_or_angle)
{ 
  ST_moving_cmd_queue_p H = NULL;
  H = (ST_moving_cmd_queue_p)malloc(sizeof(ST_moving_cmd_queue));
	H->package_id = package_id;
	H->moving_cmd = moving_cmd;
	H->v = v;
	H->w = w;
	H->s_or_angle = s_or_angle;
	H->next =NULL;
	return H;
}
/*节点插入队列链表尾部*/
static void Insert_To_Moving_Cmd_List(ST_moving_cmd_queue_p H)
{
	ST_moving_cmd_queue_p temp = moving_cmd_queue_head_p;
	 while(temp->next !=NULL)
	    {
			 temp = temp->next;
			 }
		temp->next = H;
}
/*从队列链表头部取走数据*/
static uint8_t Get_Node_From_Moving_List_Head(uint8_t *moving_cmd_p,int16_t *v_p,float *w_p,float *s_or_angle_p)
 {
	 if(moving_cmd_queue_head_p->next == NULL)
		  return 0;
	 ST_moving_cmd_queue_p H = moving_cmd_queue_head_p->next;
	 moving_cmd_queue_head_p->next = moving_cmd_queue_head_p->next->next;
	 *moving_cmd_p = H->moving_cmd;
	 *v_p = H->v;
	 *w_p = H->w;
	 *s_or_angle_p = H->s_or_angle;
	 free(H);
	 return 1;
 }
 /*获取链表尾部节点包ID*/
 static uint8_t Get_Last_Package_Id(void){
 	ST_moving_cmd_queue_p temp = moving_cmd_queue_head_p;
	 while(temp->next !=NULL)
	    {
			 temp = temp->next;
			 }
		return temp->package_id;
 }
 static uint8_t Get_queue_head_Cmd_Type(void)
 {
  if(moving_cmd_queue_head_p->next == NULL)
		  return 0;
	return moving_cmd_queue_head_p->next->moving_cmd;
 }
/*释放链表数据-不包含头节点*/
static void Free_Moving_Cmd_List(void)
{
 	ST_moving_cmd_queue_p temp = moving_cmd_queue_head_p;
	ST_moving_cmd_queue_p temp1 = NULL;
	 while(temp->next !=NULL)
	    {
				temp1 = temp->next;
			  temp->next= temp->next->next;		
        free(temp1);				
			 }
			temp->next = NULL;
}

/*释放头节点*/
static void Free_All_Head_Node(void)
{
 free(moving_cmd_queue_head_p);
}


/*判断链表是否为空*/
static uint8_t List_Is_Null(void){
 if(moving_cmd_queue_head_p->next ==NULL)
	 return 1;
 else 
	 return 0;
}

/*更新遥控命令，只有一个节点的队列*/
static void Update_Remote_Cmd(uint8_t moving_cmd,int16_t v_p,float w_p){
	if(moving_cmd_queue_head_p->next == NULL)
		  return ;
  moving_cmd_queue_head_p->next->moving_cmd = moving_cmd;
	moving_cmd_queue_head_p->next->package_id = 0;
	moving_cmd_queue_head_p->next->next = NULL;
	moving_cmd_queue_head_p->next->s_or_angle =0;
	moving_cmd_queue_head_p->next->v = v_p;
	moving_cmd_queue_head_p->next->w = w_p;
}
/*从队列链表头部取走数据*/
static uint8_t Get_Remote_Cmd(uint8_t *moving_cmd_p,int16_t *v_p,float *w_p)
 {
	 if(moving_cmd_queue_head_p->next == NULL)
		  return 0;
	 *moving_cmd_p = moving_cmd_queue_head_p->next->moving_cmd;
	 *v_p = moving_cmd_queue_head_p->next->v;
	 *w_p = moving_cmd_queue_head_p->next->w;
	 return 1;
 }
 
 
 
 /*更新轨迹跟踪命令，只有一个节点的队列*/
static void Update_Track_Tracking_Cmd(uint8_t moving_cmd,uint8_t id,int16_t v_p,float w_p,float s_or_angle_p){
	if(moving_cmd_queue_head_p->next == NULL)
		  return ;
  moving_cmd_queue_head_p->next->moving_cmd = moving_cmd;
	moving_cmd_queue_head_p->next->package_id = id;
	moving_cmd_queue_head_p->next->next = NULL;
	moving_cmd_queue_head_p->next->s_or_angle =s_or_angle_p;
	moving_cmd_queue_head_p->next->v = v_p;
	moving_cmd_queue_head_p->next->w = w_p;
}
/*从队列链表头部取走数据*/
static uint8_t Get_Track_Tracking_Cmd(uint8_t *moving_cmd_p,int16_t *v_p,float *w_p,float *s_or_angle_p)
 {
	 if(moving_cmd_queue_head_p->next == NULL)
		  return 0;
	 *moving_cmd_p = moving_cmd_queue_head_p->next->moving_cmd;
	 *v_p = moving_cmd_queue_head_p->next->v;
	 *w_p = moving_cmd_queue_head_p->next->w;
	 *s_or_angle_p = moving_cmd_queue_head_p->next->s_or_angle;
	 return 1;
 }