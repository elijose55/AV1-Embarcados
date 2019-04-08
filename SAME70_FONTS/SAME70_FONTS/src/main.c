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
	
	// Inicializa clock do perif�rico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT_PIO_ID);
	
	// Configura PIO para lidar com o pino do bot�o como entrada
	// com pull-up
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_IDX_MASK, PIO_PULLUP);
	
	// Configura interrup��o no pino referente ao botao e associa
	// fun��o de callback caso uma interrup��o for gerada
	// a fun��o de callback � a: but_callback()
	pio_handler_set(BUT_PIO,
	BUT_PIO_ID,
	BUT_PIO_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but_callback);	
  
	// Ativa interrup��o
	pio_enable_interrupt(BUT_PIO, BUT_PIO_IDX_MASK);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr�ximo de 0 maior)
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
	io_init();
	int x  = 0;
	char buffer[32];
	sprintf(buffer, "%d", x);
	
	//font_draw_text(&sourcecodepro_28, "hello", 50, 50, 1);

	while(1) {
		font_draw_text(&calibri_36, buffer, 50, 100, 1);
		font_draw_text(&arial_72, "102456", 50, 200, 2);
		if(but_flag){
			sprintf(buffer, "%d", x);
			x++;	
			but_flag = 0;
		}
		
		if (f_rtt_alarme){
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