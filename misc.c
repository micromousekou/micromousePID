
extern volatile unsigned int timer;


void wait_ms(int wtime)		//mS単位で待ち時間を生成する
{
	unsigned int start_time;
	
	start_time = timer;
	
	while( (timer - start_time) < wtime)	;

}