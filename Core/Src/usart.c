#include "usart.h"

void MX_USART1_UART_Init(void)
{
    GPIO_InitTypeDef gpio_config = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    gpio_config.Pin = GPIO_PIN_9;
    gpio_config.Mode = GPIO_MODE_AF_PP;
    gpio_config.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_config);

    gpio_config.Pin = GPIO_PIN_10;
    gpio_config.Mode = GPIO_MODE_INPUT;
    gpio_config.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio_config);

    USART1->BRR = HAL_RCC_GetPCLK2Freq() / 100000U;
    USART1->CR1 = USART_CR1_M | USART_CR1_PCE | USART_CR1_RE |
                  USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_IDLEIE |
                  USART_CR1_UE;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;

    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}
