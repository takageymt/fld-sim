/*↓↓↓ヘッダファイル↓↓↓*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <eggx.h>
/*↑↑↑ヘッダファイル↑↑↑*/

/*↓↓↓定数宣言↓↓↓*/
#define Field_X   64                    //空間 x分割数
#define Field_Y   34                    //空間 y分割数
#define Field_Div (Field_X * Field_Y)   //空間 総分割数
#define Fluid_X   (Field_X - 2)         //流体 x分割数
#define Fluid_Y   (Field_Y - 2)         //流体 y分割数

#define Delta_t   0.59                  //tの変化量   
#define Re        (1.0 / 1023.1)        //レイノルズ数 Re
#define Omega     (2.0 - 5.0 / Field_X) //加速係数 ω 1.9

#define V_Lower   -3.0                  //速度下限
#define V_Upper   3.0                   //速度上限

#define Plt_Max   6000                   //汚染物質最大数

#define Error 0.0047                //圧力計算での許容誤差

#define SPD 7
/*↑↑↑定数宣言↑↑↑*/

/*↓↓↓変数宣言↓↓↓*/
double vx[Field_X][Field_Y];      //格子 x速度
double vy[Field_X][Field_Y];      //格子 y速度
double vx_temp[Field_X][Field_Y]; //vx退避用
double vy_temp[Field_X][Field_Y]; //vy退避用

double ss[Field_X][Field_Y];      //発散
double p[Field_X][Field_Y];       //圧力
double p_temp[Field_X][Field_Y];  //p 退避用

double wall_v = 0.0;

int    pump = 3;                  //流量
double flow;                      

double plt_x[Plt_Max];            //汚染物質 x座標
double plt_y[Plt_Max];            //汚染物質 y座標

int    info[Field_X][Field_Y];    //ステージ情報

int    frame;

int win; //NEW
/*↑↑↑変数宣言↑↑↑*/

/*↓↓↓関数プロトタイプ宣言↓↓↓*/
void initialize( void ); //初期化関数
void Advection( void );  //移流計算関数
void ExForce( void );    //外力計算関数
void WallVRiv( void );   //壁速度修正関数
void Divergence( void ); //発散計算関数
void SOR( void );        //圧力計算関数
void FluidVRiv( void );  //流体速度修正関数

void makeObject( int sx, int sy, int tx, int ty); //障害物生成関数

void MovePlt( void );    //汚染物質移動関数
void NewPlt( double *x, double *y ); //汚染物質作成関数

double limit( double value, double min, double max ); //上下限関数
/*↑↑↑関数プロトタイプ宣言↑↑↑*/

/*↓↓↓メイン関数↓↓↓*/
int main( void )
{
  gsetinitialattributes(DISABLE, BOTTOM_LEFT_ORIGIN); //NEW

  win = gopen(Field_X*SPD, Field_Y*SPD); //NEW
  layer(win, 0, 1); //NEW

  initialize(); //初期化
  
  frame = 0;

  srand((unsigned)time(NULL));

  while ( frame != 10000) {    
    flow = 0.399 * Delta_t * pump / 1;
  
    gclr(win); //NEW
    
    Advection(); //移流
    
    ExForce(); //外力
    
    WallVRiv(); //壁速度固定

    Divergence(); //ダイバージェンス計算
    
    SOR(); //圧力計算
    
    FluidVRiv(); //流体速度修正

    MovePlt();
    
    frame++;
    
    copylayer(win, 1, 0); //NEW
    msleep(100); //NEW
  }

  ggetch(); //NEW
  gclose(win); //NEW

  return 0;
}
/*↑↑↑メイン関数↑↑↑*/

/*↓↓↓障害物生成関数↓↓↓*/
void makeObject( int sx, int sy, int tx, int ty)
{
  int i, j;

  for ( i = sx; i < tx; i++ ) {
    for ( j = sy; j < ty; j++ ) {
      info[i][j] = 1;
    }
  }

}
/*↑↑↑障害物生成関数↑↑↑*/

/*↓↓↓初期化関数↓↓↓*/
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
/*↑↑↑初期化関数↑↑↑*/

/*↓↓↓移流計算関数↓↓↓*/
void Advection( void ) 
{
  int i, j; //ループ変数
  double u, v; //x速度 y速度

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
/*↑↑↑移流計算関数↑↑↑*/

/*↓↓↓外力計算関数↓↓↓*/
void ExForce( void )
{
  return;
}
/*↑↑↑外力計算関数↑↑↑*/

/*↓↓↓壁速度修正関数↓↓↓*/
void WallVRiv( void )
{
  int i, j; //ループ変数

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
/*↑↑↑壁速度修正関数↑↑↑*/

/*↓↓↓発散計算関数↓↓↓*/
void Divergence( void )
{
  int i, j; //ループ変数
  
  for ( i = 1; i <= Fluid_X; i++ ) { //流体範囲 x方向
    for ( j = 1; j <= Fluid_Y; j++ ) { //流体範囲 y方向 
      ss[i][j] = vx[i][j] + vy[i][j] - vx[i-1][j] - vy[i][j-1] - flow*(info[i][j]==2);
    } 
  }

  return;
}
/*↑↑↑発散計算関数↑↑↑*/

/*↓↓↓圧力計算関数↓↓↓*/
void SOR( void )
{
  int l, i, j; //ループ変数
  
  int top, left, right, bottom;
  double p_top, p_left, p_right, p_bottom;
 
  int convergence = -1;
  
  for ( l = 0; l < 15+Field_Div*(frame<6)*(frame!=0) && convergence != 1; l++ ){
    for ( i = 1; i <= Fluid_X; i++ ) {
      for ( j = 1; j <= Fluid_Y; j++ ) {
	top    = (info[i][j-1]!=1); //上の格子が壁か否か
	left   = (info[i-1][j]!=1); //左の格子が壁か否か
	right  = (info[i+1][j]!=1); //右の格子が壁か否か
	bottom = (info[i][j+1]!=1); //下の格子が壁か否か
		
	p_top    = p[i][j-1] * top;    //上の格子の圧力（壁なら０）
	p_left   = p[i-1][j] * left;   //左の格子の圧力（壁なら０）
	p_right  = p[i+1][j] * right;  //右の格子の圧力（壁なら０）
	p_bottom = p[i][j+1] * bottom; //下の格子の圧力（壁なら０）
       
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
/*↑↑↑圧力計算関数↑↑↑*/

/*↓↓↓流体速度修正関数↓↓↓*/
void FluidVRiv( void )
{
  int i, j; //ループ変数
  double vorticity; //NEW

  for ( i = 1; i <= Fluid_X; i++ ) {
    for ( j = 1; j <= Fluid_Y; j++ ) {
      if ( info[i][j] != 1) {
	if ( info[i+1][j] != 1 ) {
	  vx[i][j] -= p[i+1][j] - p[i][j]; //x速度に圧力影響を与える
	}
	if ( info[i][j+1] != 1 ) {
	  vy[i][j] -= p[i][j+1] - p[i][j]; //y速度に圧力影響を与える
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
/*↑↑↑流体速度修正関数↑↑↑*/

/*↓↓↓汚染物質移動関数↓↓↓*/
void MovePlt( void )
{
  int next_x, next_y;
  int occurre_plt;
  
  double re_fine_x, re_fine_y; //整数座標上のより細かい相対座標

  int i;

  newpen(win, 5);

  occurre_plt = 5*pump;

  for ( i = 0; i < Plt_Max; i++) {
    next_x = (int)limit(plt_x[i] + 1.0, 0.0, Field_X - 1); //x方向の移動先
    next_y = (int)limit(plt_y[i] + 1.0, 0.0, Field_Y - 1); //y方向の移動先
    re_fine_x = fmod(plt_x[i], 1.0); //格子内でのx座標
    re_fine_y = fmod(plt_y[i], 1.0); //格子内でのy座標
    
    if( info[next_x][next_y] != 0 ) {
      
      if ( occurre_plt != 0) {
	NewPlt( &plt_x[i], &plt_y[i]);
	occurre_plt--;
      }

    } else {
      
      plt_x[i] += (vx[next_x][next_y]*re_fine_x - vx[next_x-1][next_y]*(re_fine_x-1.0)) * Delta_t; //x移動
      plt_y[i] += (vy[next_x][next_y]*re_fine_y - vy[next_x][next_y-1]*(re_fine_y-1.0)) * Delta_t; //y移動
  
      pset(win, plt_x[i]*SPD, plt_y[i]*SPD); //NEW
    }
  }

  return;
}
/*↑↑↑汚染物質移動関数↑↑↑*/


/*↓↓↓汚染物質作成関数↓↓↓*/
void NewPlt( double *x, double *y )
{
  *x = 1.1;
  *y = 0.435 * Field_Y + 0.0013 * (rand()%3820);

  return;
}
/*↑↑↑汚染物質作成関数↑↑↑*/


/*↓↓↓上下限関数↓↓↓*/
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
/*↑↑↑上下限関数↑↑↑*/
