LWIP -> udp.c 760
    changed
        always true
    if (pbuf_add_header(p, UDP_HLEN)) -> if (1)

HAL -> stm32f7xx_hal_eth.c -> 895
    added
        UDP loss dicreased almost 100%
    __HAL_UNLOCK(heth);

HAL -> stm32f7xx_it.c -> TIM2_IRQHandler(void)
    added
        timer callback routine time decreased to 400ns
        optimization -Ofast
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	GPIOF->ODR = code[code_index];
	code_index++;
	if (code_index == 6) HAL_TIM_Base_Stop_IT(&htim2);

    removed
    HAL_TIM_IRQHandler(&htim2);

HAL -> stm32f7xx_hal_dma.c -> 464
    added
        fixed DMA HAL_LOCKED bug
    __HAL_UNLOCK(hdma);

HAL -> stm32f7xx_hal_dma.c -> 470
    added
        fixed HAL_DMA_STATE_ABORT state bug
    if (hdma->State == HAL_DMA_STATE_ABORT) hdma->State = HAL_DMA_STATE_READY;

IQ_out DMA2 Channel 4, TIM1 Channel 4
AB_out DMA2 Channel 6, TIM1 Channel 3
TIM2 coder 400ns
TIM3 coder 110ms
