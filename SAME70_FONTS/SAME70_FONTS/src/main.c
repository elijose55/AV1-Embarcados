/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include <asf.h>
#include "tfont.h"
#include "sourcecodepro_28.h"
#include "calibri_36.h"
#include "arial_72.h"


/************************************************************************/
/* defines                                                              */
/************************************************************************/

//pb1
#define BUT_PIO           PIOB
#define BUT_PIO_ID        11
#define BUT_PIO_IDX       1u
#define BUT_PIO_IDX_MASK  (1u << BUT_PIO_IDX)

#define LED_PIO       PIOC
#define LED_PIO_ID    ID_PIOC
#define LED_IDX       8u
#define LED_IDX_MASK  (1u << LED_IDX)


#define YEAR        2018
#define MOUNTH      3
#define DAY         19
#define WEEK        12
#define HOUR        15
#define MINUTE      45
#define SECOND      0

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/
volatile Bool f_rtt_alarme = false;

volatile int but_flag = 0;

/************************************************************************/
/* handler / callbacks                                                  */
/************************************************************************/

void but_callback(void)
{
	but_flag = 1;
}

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/
void pin_toggle(Pio *pio, uint32_t mask);
void io_init(void);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
void RTC_init(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		pin_toggle(LED_PIO, LED_IDX_MASK);    // BLINK Led
		f_rtt_alarme = true;                  // flag RTT alarme
	}
}

/**
* \brief Interrupt handler for the RTC. Refresh the display.
*/
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
		
		
		rtc_get_time(ID_RTC);
	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}

struct ili9488_opt_t g_ili9488_display_opt;

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/




//rtt
void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void io_init(void){
	/* led */
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
	
	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT_PIO_ID);
	
	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_IDX_MASK, PIO_PULLUP);
	
	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT_PIO,
	BUT_PIO_ID,
	BUT_PIO_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but_callback);	
  
	// Ativa interrupção
	pio_enable_interrupt(BUT_PIO, BUT_PIO_IDX_MASK);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4); // Prioridade 4  
}

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}

//lcd

void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	
}


void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}	
}

double calcula_velocidade(int n){
	double velocidade = 2*3.14*n/4;
	return velocidade;
}

double calcula_distancia(int n){
	double distancia = 2*3.14*0.325*n;
	return distancia;
}

//rtc

/**
* Configura o RTC para funcionar com interrupcao de alarme
*/
void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC, RTC_IER_ALREN);

}



/************************************************************************/
/* Main                                                                 */
/************************************************************************/

int main(void) {
	
	board_init();
	sysclk_init();	
	configure_lcd();
	
	// Desativa watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	// configura botao com interrupcao
	
	// Inicializa RTT com IRQ no alarme.
	f_rtt_alarme = true;
	
	/** Configura RTC */
	RTC_init();
	
	/* configura alarme do RTC */
	rtc_set_date_alarm(RTC, 1, MOUNTH, 1, DAY);
	rtc_set_time_alarm(RTC, 1, HOUR, 1, MINUTE, 1, SECOND+1);
	
	io_init();
	int x = 0;
	int y = 0;
	int velocidade = 0;
	int distancia = 0;
	char buffer[32];
	char buffer_vel[32];
	char buffer_dis[32];
	sprintf(buffer, "%d", x);
	sprintf(buffer_vel, "%02d", velocidade);
	sprintf(buffer_dis, "%02d", distancia);
	
	//font_draw_text(&sourcecodepro_28, "hello", 50, 50, 1);

	while(1) {
		sprintf(buffer, "%02d", x);
		//sprintf(buffer_vel, "%d", velocidade);
		font_draw_text(&calibri_36, buffer, 50, 20, 1);
		font_draw_text(&calibri_36, "Velocidade (m/s):", 50, 60, 1);
		font_draw_text(&arial_72, buffer_vel, 50, 110, 2);
		font_draw_text(&calibri_36, "Distancia (m):", 50, 190, 1);
		font_draw_text(&arial_72, buffer_dis, 50, 240, 2);
		if(but_flag){

			x++;	
			y++;
			but_flag = 0;
		}
		
		if (f_rtt_alarme){
			velocidade = calcula_velocidade(x);
			distancia = calcula_distancia(y);
			x = 0;
			sprintf(buffer_vel, "%02d", velocidade);
			sprintf(buffer_dis, "%02d", distancia);
      
		  /*
		   * O clock base do RTT é 32678Hz
		   * Para gerar outra base de tempo é necessário
		   * usar o PLL pre scale, que divide o clock base.
		   *
		   * Nesse exemplo, estamos operando com um clock base
		   * de pllPreScale = 32768/32768/2 = 2Hz
		   *
		   * Quanto maior a frequência maior a resolução, porém
		   * menor o tempo máximo que conseguimos contar.
		   *
		   * Podemos configurar uma IRQ para acontecer quando 
		   * o contador do RTT atingir um determinado valor
		   * aqui usamos o irqRTTvalue para isso.
		   * 
		   * Nesse exemplo o irqRTTvalue = 8, causando uma
		   * interrupção a cada 2 segundos (lembre que usamos o 
		   * pllPreScale, cada incremento do RTT leva 500ms (2Hz).
		   */
		  uint16_t pllPreScale = (int) (((float) 32768) / 2.0);
		  uint32_t irqRTTvalue  = 4;
      
		  // reinicia RTT para gerar um novo IRQ
		  RTT_init(pllPreScale, irqRTTvalue);   
		  

      
		 /*
		  * caso queira ler o valor atual do RTT, basta usar a funcao
		  *   rtt_read_timer_value()
		  */
      
		  /*
		   * CLEAR FLAG
		   */
		  f_rtt_alarme = false;
		}
		
		
	}
}