/*�������إå��ե����뢭����*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <eggx.h>
/*�������إå��ե����뢬����*/

/*������������������*/
#define Field_X   64                    //���� xʬ���
#define Field_Y   34                    //���� yʬ���
#define Field_Div (Field_X * Field_Y)   //���� ��ʬ���
#define Fluid_X   (Field_X - 2)         //ή�� xʬ���
#define Fluid_Y   (Field_Y - 2)         //ή�� yʬ���

#define Delta_t   0.59                  //t���Ѳ���   
#define Re        (1.0 / 1023.1)        //�쥤�Υ륺�� Re
#define Omega     (2.0 - 5.0 / Field_X) //��®���� �� 1.9

#define V_Lower   -3.0                  //®�ٲ���
#define V_Upper   3.0                   //®�پ��

#define Plt_Max   6000                   //����ʪ�������

#define Error 0.0047                //���Ϸ׻��Ǥε��Ƹ�

#define SPD 7
/*������������������*/

/*�������ѿ����������*/
double vx[Field_X][Field_Y];      //�ʻ� x®��
double vy[Field_X][Field_Y];      //�ʻ� y®��
double vx_temp[Field_X][Field_Y]; //vx������
double vy_temp[Field_X][Field_Y]; //vy������

double ss[Field_X][Field_Y];      //ȯ��
double p[Field_X][Field_Y];       //����
double p_temp[Field_X][Field_Y];  //p ������

double wall_v = 0.0;

int    pump = 3;                  //ή��
double flow;                      

double plt_x[Plt_Max];            //����ʪ�� x��ɸ
double plt_y[Plt_Max];            //����ʪ�� y��ɸ

int    info[Field_X][Field_Y];    //���ơ�������

int    frame;

int win; //NEW
/*�������ѿ����������*/

/*�������ؿ��ץ�ȥ��������������*/
void initialize( void ); //������ؿ�
void Advection( void );  //��ή�׻��ؿ�
void ExForce( void );    //���Ϸ׻��ؿ�
void WallVRiv( void );   //��®�ٽ����ؿ�
void Divergence( void ); //ȯ���׻��ؿ�
void SOR( void );        //���Ϸ׻��ؿ�
void FluidVRiv( void );  //ή��®�ٽ����ؿ�

void makeObject( int sx, int sy, int tx, int ty); //�㳲ʪ�����ؿ�

void MovePlt( void );    //����ʪ����ư�ؿ�
void NewPlt( double *x, double *y ); //����ʪ�������ؿ�

double limit( double value, double min, double max ); //�岼�´ؿ�
/*�������ؿ��ץ�ȥ��������������*/

/*�������ᥤ��ؿ�������*/
int main( void )
{
  gsetinitialattributes(DISABLE, BOTTOM_LEFT_ORIGIN); //NEW

  win = gopen(Field_X*SPD, Field_Y*SPD); //NEW
  layer(win, 0, 1); //NEW

  initialize(); //�����
  
  frame = 0;

  srand((unsigned)time(NULL));

  while ( frame != 10000) {    
    flow = 0.399 * Delta_t * pump / 1;
  
    gclr(win); //NEW
    
    Advection(); //��ή
    
    ExForce(); //����
    
    WallVRiv(); //��®�ٸ���

    Divergence(); //�����С������󥹷׻�
    
    SOR(); //���Ϸ׻�
    
    FluidVRiv(); //ή��®�ٽ���

    MovePlt();
    
    frame++;
    
    copylayer(win, 1, 0); //NEW
    msleep(100); //NEW
  }

  ggetch(); //NEW
  gclose(win); //NEW

  return 0;
}
/*�������ᥤ��ؿ�������*/

/*�������㳲ʪ�����ؿ�������*/
void makeObject( int sx, int sy, int tx, int ty)
{
  int i, j;

  for ( i = sx; i < tx; i++ ) {
    for ( j = sy; j < ty; j++ ) {
      info[i][j] = 1;
    }
  }

}
/*�������㳲ʪ�����ؿ�������*/

/*������������ؿ�������*/
void initialize( void )
{
  int i, j;
  
  for ( i = 0; i < Field_X; i++ ) {
    for ( j = 0; j < Field_Y; j++ ) {
      if ( i == 0 || j == 0 || i == Field_X-1 || j == Field_Y-1 ) {
	info[i][j] = 1;
      } else if ( i == 1 ) {
	info[i][j] = 2;
      } else if ( i == Field_X-2 ) {
	info[i][j] = 3;
      } else {
	info[i][j] = 0;
      }
    }
  }

  makeObject(10, 15, 12, 20);
  makeObject(15, 13, 16, 18);

}
/*������������ؿ�������*/

/*��������ή�׻��ؿ�������*/
void Advection( void ) 
{
  int i, j; //�롼���ѿ�
  double u, v; //x®�� y®��

  memcpy(vx_temp, vx, sizeof(vx)); 
  memcpy(vy_temp, vy, sizeof(vy));     

  for ( i = 1; i <= Fluid_X; i++ ) {
    for ( j = 1; j <= Fluid_Y; j++ ) {
      v = (vy_temp[i][j] + vy_temp[i][j-1] + vy_temp[i+1][j] + vy_temp[i+1][j-1]) / 4.0;           
      u = (vx_temp[i][j] + vx_temp[i-1][j] + vx_temp[i][j+1] + vx_temp[i-1][j+1]) / 4.0;             
      
      if (vx[i][j] >= 0 && v >= 0) {
	vx[i][j] = vx[i][j] - limit((vx[i][j]*(vx[i][j]-vx_temp[i-1][j]) + v*(vx[i][j]-vx_temp[i][j-1]) + Re*(vx_temp[i+1][j]+vx_temp[i-1][j]+vx_temp[i][j+1]+vx_temp[i][j-1]-vx[i][j]*4.0)) * Delta_t, V_Lower, V_Upper);
      } else if (vx[i][j] < 0 && v >= 0) {
      	vx[i][j] = vx[i][j] - limit((vx[i][j]*(vx_temp[i+1][j]-vx[i][j]) + v*(vx[i][j]-vx_temp[i][j-1]) + Re*(vx_temp[i+1][j]+vx_temp[i-1][j]+vx_temp[i][j+1]+vx_temp[i][j-1]-vx[i][j]*4.0)) * Delta_t, V_Lower, V_Upper);
      } else if (vx[i][j] >= 0 && v < 0) {
      	vx[i][j] = vx[i][j] - limit((vx[i][j]*(vx[i][j]-vx_temp[i-1][j]) + v*(vx_temp[i][j+1]-vx[i][j]) + Re*(vx_temp[i+1][j]+vx_temp[i-1][j]+vx_temp[i][j+1]+vx_temp[i][j-1]-vx[i][j]*4.0)) * Delta_t, V_Lower, V_Upper);
      } else if (vx[i][j] < 0 && v < 0) {
      	vx[i][j] = vx[i][j] - limit((vx[i][j]*(vx_temp[i+1][j]-vx[i][j]) + v*(vx_temp[i][j+1]-vx[i][j]) + Re*(vx_temp[i+1][j]+vx_temp[i-1][j]+vx_temp[i][j+1]+vx_temp[i][j-1]-vx[i][j]*4.0)) * Delta_t, V_Lower, V_Upper);
      }
      
      if (u >= 0 && vy[i][j] >= 0) {
	vy[i][j] = vy[i][j] - limit((vy[i][j]*(vy[i][j]-vy_temp[i][j-1]) + u*(vy[i][j]-vy_temp[i-1][j]) + Re*(vy_temp[i+1][j]+vy_temp[i-1][j]+vy_temp[i][j+1]+vy_temp[i][j-1]-vy[i][j]*4.0)) * Delta_t, V_Lower, V_Upper);
      } else if (u < 0 && vy[i][j] >= 0) {
	vy[i][j] = vy[i][j] - limit((vy[i][j]*(vy[i][j]-vy_temp[i][j-1]) + u*(vy_temp[i+1][j]-vy[i][j]) + Re*(vy_temp[i+1][j]+vy_temp[i-1][j]+vy_temp[i][j+1]+vy_temp[i][j-1]-vy[i][j]*4.0)) * Delta_t, V_Lower, V_Upper);
      } else if (u >= 0 && vy[i][j] < 0) {
	vy[i][j] = vy[i][j] - limit((vy[i][j]*(vy_temp[i][j+1]-vy[i][j]) + u*(vy[i][j]-vy_temp[i-1][j]) + Re*(vy_temp[i+1][j]+vy_temp[i-1][j]+vy_temp[i][j+1]+vy_temp[i][j-1]-vy[i][j]*4.0)) * Delta_t, V_Lower, V_Upper);
      } else if (u < 0 && vy[i][j] < 0) {
	vy[i][j] = vy[i][j] - limit((vy[i][j]*(vy_temp[i][j+1]-vy[i][j]) + u*(vy_temp[i+1][j]-vy[i][j]) + Re*(vy_temp[i+1][j]+vy_temp[i-1][j]+vy_temp[i][j+1]+vy_temp[i][j-1]-vy[i][j]*4.0)) * Delta_t, V_Lower, V_Upper);
      }
      
    }
  }

  return;
}
/*��������ή�׻��ؿ�������*/

/*���������Ϸ׻��ؿ�������*/
void ExForce( void )
{
  return;
}
/*���������Ϸ׻��ؿ�������*/

/*��������®�ٽ����ؿ�������*/
void WallVRiv( void )
{
  int i, j; //�롼���ѿ�

  for ( i = 0; i < Field_X; i++ ) {
    for ( j = 0; j < Field_Y; j++ ) {
      if ( info[i][j] == 1 ) {
	vx[i][j] = wall_v;
	vy[i][j] = wall_v;
	if ( i != 0 ) {
	  vx[i-1][j] = wall_v;
	}
	if ( j != 0 ) {
	  vy[i][j-1] = wall_v;
	}
      }
    }
  }
  
  return;
}
/*��������®�ٽ����ؿ�������*/

/*������ȯ���׻��ؿ�������*/
void Divergence( void )
{
  int i, j; //�롼���ѿ�
  
  for ( i = 1; i <= Fluid_X; i++ ) { //ή���ϰ� x����
    for ( j = 1; j <= Fluid_Y; j++ ) { //ή���ϰ� y���� 
      ss[i][j] = vx[i][j] + vy[i][j] - vx[i-1][j] - vy[i][j-1] - flow*(info[i][j]==2);
    } 
  }

  return;
}
/*������ȯ���׻��ؿ�������*/

/*���������Ϸ׻��ؿ�������*/
void SOR( void )
{
  int l, i, j; //�롼���ѿ�
  
  int top, left, right, bottom;
  double p_top, p_left, p_right, p_bottom;
 
  int convergence = -1;
  
  for ( l = 0; l < 15+Field_Div*(frame<6)*(frame!=0) && convergence != 1; l++ ){
    for ( i = 1; i <= Fluid_X; i++ ) {
      for ( j = 1; j <= Fluid_Y; j++ ) {
	top    = (info[i][j-1]!=1); //��γʻҤ��ɤ��ݤ�
	left   = (info[i-1][j]!=1); //���γʻҤ��ɤ��ݤ�
	right  = (info[i+1][j]!=1); //���γʻҤ��ɤ��ݤ�
	bottom = (info[i][j+1]!=1); //���γʻҤ��ɤ��ݤ�
		
	p_top    = p[i][j-1] * top;    //��γʻҤΰ��ϡ��ɤʤ飰��
	p_left   = p[i-1][j] * left;   //���γʻҤΰ��ϡ��ɤʤ飰��
	p_right  = p[i+1][j] * right;  //���γʻҤΰ��ϡ��ɤʤ飰��
	p_bottom = p[i][j+1] * bottom; //���γʻҤΰ��ϡ��ɤʤ飰��
       
	p[i][j] = (((Omega/4.0)*(4-top-left-right-bottom) + (1.0-Omega)) * p[i][j] + (Omega/4.0)*(p_top+p_left+p_right+p_bottom - ss[i][j]))*(info[i][j]!=3);

      }
    }
    if ( l % 5 == 3 ) {
      memcpy(p_temp, p, sizeof(p));
    } else if ( l % 5 == 4 ) {
      convergence = 1;
      for ( i = 0; i < Field_X / 2; i++ ) {
	for ( j = 0; j < Field_Y / 2; j++ ) {
	  if ( fabs(p[i*2][j*2] - p_temp[i*2][j*2]) > Error ) {
	    convergence = 0;
	  }
	}
      }
    }
  }
  
  return;
}
/*���������Ϸ׻��ؿ�������*/

/*������ή��®�ٽ����ؿ�������*/
void FluidVRiv( void )
{
  int i, j; //�롼���ѿ�
  double vorticity; //NEW

  for ( i = 1; i <= Fluid_X; i++ ) {
    for ( j = 1; j <= Fluid_Y; j++ ) {
      if ( info[i][j] != 1) {
	if ( info[i+1][j] != 1 ) {
	  vx[i][j] -= p[i+1][j] - p[i][j]; //x®�٤˰��ϱƶ���Ϳ����
	}
	if ( info[i][j+1] != 1 ) {
	  vy[i][j] -= p[i][j+1] - p[i][j]; //y®�٤˰��ϱƶ���Ϳ����
	}
      }
    }
  }

  /*NEW*/
  for ( i = 1; i <= Fluid_X; i++ ) {
    for ( j = 1; j <= Fluid_Y; j++ ) {
      vorticity = 160.0*(vx[i-1][j-1]+vx[i][j-1]-vx[i-1][j+1]-vx[i][j+1]-vy[i-1][j-1]-vy[i-1][j]+vy[i+1][j-1]+vy[i+1][j]);
      newrgbcolor(win, (int)limit(vorticity, 0.0, 255.0), (int)limit(-vorticity, 0.0, 255.0), 0);
      if ( info[i][j] == 1 ) {
	newpen(win, 1);
      }
      fillrect(win, (double)(i-1)*SPD, (double)(j-1)*SPD, 2.0*SPD, 2.0*SPD);
    }
  }
  /*NEW*/

  return;
}
/*������ή��®�ٽ����ؿ�������*/

/*����������ʪ����ư�ؿ�������*/
void MovePlt( void )
{
  int next_x, next_y;
  int occurre_plt;
  
  double re_fine_x, re_fine_y; //������ɸ��Τ��٤������к�ɸ

  int i;

  newpen(win, 5);

  occurre_plt = 5*pump;

  for ( i = 0; i < Plt_Max; i++) {
    next_x = (int)limit(plt_x[i] + 1.0, 0.0, Field_X - 1); //x�����ΰ�ư��
    next_y = (int)limit(plt_y[i] + 1.0, 0.0, Field_Y - 1); //y�����ΰ�ư��
    re_fine_x = fmod(plt_x[i], 1.0); //�ʻ���Ǥ�x��ɸ
    re_fine_y = fmod(plt_y[i], 1.0); //�ʻ���Ǥ�y��ɸ
    
    if( info[next_x][next_y] != 0 ) {
      
      if ( occurre_plt != 0) {
	NewPlt( &plt_x[i], &plt_y[i]);
	occurre_plt--;
      }

    } else {
      
      plt_x[i] += (vx[next_x][next_y]*re_fine_x - vx[next_x-1][next_y]*(re_fine_x-1.0)) * Delta_t; //x��ư
      plt_y[i] += (vy[next_x][next_y]*re_fine_y - vy[next_x][next_y-1]*(re_fine_y-1.0)) * Delta_t; //y��ư
  
      pset(win, plt_x[i]*SPD, plt_y[i]*SPD); //NEW
    }
  }

  return;
}
/*����������ʪ����ư�ؿ�������*/


/*����������ʪ�������ؿ�������*/
void NewPlt( double *x, double *y )
{
  *x = 1.1;
  *y = 0.435 * Field_Y + 0.0013 * (rand()%3820);

  return;
}
/*����������ʪ�������ؿ�������*/


/*�������岼�´ؿ�������*/
double limit( double value, double min, double max )
{
  if ( value < min) {
    return min;
  } else if ( value > max ) {
    return max;
  } else {
    return value;
  }
}
/*�������岼�´ؿ�������*/
