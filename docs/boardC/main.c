/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"
#include <string.h>
#include "lwip/sockets.h" // �뚯폆 �⑥닔�� (socket, sendto ��)
#include "lwip/inet.h" // IP 二쇱냼 蹂��� (inet_addr)

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 1. ��(Queue)�� �ｌ쓣 �대� �곗씠�� 援ъ“泥�
typedef struct {
    uint16_t id;      // CAN ID
    uint8_t  dlc;
    uint8_t  data[8]; // �쇱꽌 �곗씠��
} CanQueueItem;

// 2. �대뜑�� �꾩넚�� �⑦궥
typedef struct __attribute__((packed)) {
    uint8_t  start_byte;
    uint16_t can_id;
    uint8_t  dlc;
    uint8_t  data[8];
} UdpPacket;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for EthernetTask */
osThreadId_t EthernetTaskHandle;
const osThreadAttr_t EthernetTask_attributes = {
  .name = "EthernetTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* USER CODE BEGIN PV */
static CAN_RxHeaderTypeDef RxHeader;
static uint8_t RxData[8];

extern struct netif gnetif;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void StartDefaultTask(void *argument);
void StartEthernetTask(void *argument);

/* USER CODE BEGIN PFP */
static void CAN_Filter_Allow_All_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// �쇱꽌 �곗씠�� 蹂닿���
volatile uint8_t val_ultrasonic = 0; // 珥덉쓬��
volatile uint8_t val_motor = 0; // 모터
volatile uint8_t val_temp = 0;       // �⑤룄
volatile uint8_t val_humi = 0;       // �듬룄
volatile uint8_t val_button = 0;     // 踰꾪듉
volatile uint8_t val_led = 0;        // LED �곹깭

// �대뜑�� �꾩넚�� �꾪븳 源껊컻 (�대뼡 ID媛� �붾뒗吏� �쒖떆)
volatile uint16_t received_can_id = 0;
volatile uint8_t  received_can_data[8];
volatile uint8_t  is_data_received = 0; // �곗씠�� �붿쓬!
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  // 1) �꾪꽣: 0x103留� 諛쏄린(�먰븯硫� ALL PASS濡� 諛붽퓭�� ��)
  CAN_Filter_Allow_All_Init();

  // 2) CAN Start
  if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();

  // 3) �섏떊 �명꽣�쏀듃 �쒖꽦��
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    Error_Handler();



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(CanQueueItem), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of EthernetTask */
  EthernetTaskHandle = osThreadNew(StartEthernetTask, NULL, &EthernetTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin); // heartbeat
	  HAL_Delay(1000);

	  osDelay(1000);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// CAN �섏떊 �명꽣�쏀듃 肄쒕갚
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance != CAN1) return;

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
      CanQueueItem item;
      item.id = RxHeader.StdId;
      item.dlc = RxHeader.DLC;
      memset(item.data, 0, 8);
      memcpy(item.data, RxData, item.dlc);

      // �� �대쫫 �섏젙�� (myQueue01Handle)
      osMessageQueuePut(myQueue01Handle, &item, 0, 0);

      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
  }
}

// [4�④퀎] �대뜑�� 泥섎━ �⑥닔 (�먯뿉�� 爰쇰궡�� 蹂대궡湲�)
//void Process_Ethernet_Queue(void)
//{
//	// 1. �먭� 鍮꾩뼱�덈뒗吏� �뺤씤
//	if (Is_Queue_Empty()) return;
//
//	// 2. �먯뿉�� �곗씠�� 爰쇰궡湲� (dequeue)
//	unit8_t data = Dequeue_Data();
//}


// �꾪꽣 �ㅼ젙 (�꾩껜 �덉슜�쇰줈 蹂�寃� 異붿쿇)
static void CAN_Filter_Allow_All_Init(void)
{
  CAN_FilterTypeDef f = {0};
  f.FilterBank = 0;
  f.FilterMode = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_32BIT;

  f.FilterIdHigh = 0x0000;
  f.FilterIdLow  = 0x0000;
  f.FilterMaskIdHigh = 0x0000; // 留덉뒪�� 0 = 紐⑤뱺 ID �듦낵
  f.FilterMaskIdLow  = 0x0000;

  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation = ENABLE;
  f.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) Error_Handler();
}

//void StartEthernetTask(void *argument)
//{
//
//	// 1. LwIP 珥덇린�� ��湲�
//	osDelay(1000);
//
//	// 2. �뚯폆 �앹꽦
//	int sock = socket(AF_INET, SOCK_DGRAM,0);
//
//	struct sockaddr_in dest_addr;
//	dest_addr.sin_family = AF_INET;
//	dest_addr.sin_port  = htons(8080);
//	dest_addr.sin_addr.s_addr = inet_addr("10,10,14,79");
//
//	CanQueueItem recv_item;	// �먯뿉�� 爰쇰궡 �댁쓣 蹂���
//	UdpPacket packet; 		// �대뜑�룹쑝濡� 蹂대궪 �⑦궥 援ъ“泥�
//
//	for(;;)
//	{	// 3. �먯뿉�� �곗씠�� 爰쇰궡湲� (�곗씠�� �� �뚭퉴吏� ��湲�)
//		if(osMessageQueueGet(myQueue01Handle, &recv_item, NULL, osWaitForever) ==osOK)
//		{
//			// 4. �ㅼ쐞移섎Ц�쇰줈 �곗씠�� 遺꾨쪟 諛� ����
//			switch (recv_item.id)
//			{
//			   case 0x103: // 珥덉쓬��
//			     val_ultrasonic = recv_item.data[0];
//			     break;
//			   case 0x104: // �⑤룄
//			     val_temp = recv_item.data[0];
//			     break;
//			   case 0x105: // �듬룄
//			     val_humi = recv_item.data[0];
//			     break;
//			   case 0x201: // 踰꾪듉
//			     val_button = recv_item.data[0];
//			     break;
//			   case 0x202: // LED
//			     val_led = recv_item.data[0];
//			     break;
//			   default:
//			     // �뺤쓽�섏� �딆� ID 泥섎━ (�꾩슂��)
//			   break;
//			 }
//
//			// 5. �대뜑�� �⑦궥 援ъ“泥� 梨꾩슦湲� (Packing)
//			packet.start_byte = 0xAA;		// �ㅻ뜑
//			packet.can_id = recv_item.id;   // CAN ID (�대뼡 �쇱꽌�몄� PC�� �뚯븘�� �섎�濡�)
//			packet.dlc = 8;					// 湲몄씠
//			memcpy(packet.data, recv_item.data, 8); // �곗씠�� 蹂듭궗
//
//			// 6. UDP �꾩넚 (Pi濡� �꾩넚)
//			sendto(sock, (char*)&packet, sizeof(packet), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
//
//			// �꾩넚 �뺤씤 LED (珥덈줉��)
//			HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//		}
//	}
//}




/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    // LwIP �ㅽ깮�� �대��곸쑝濡� �뚯븘媛��� �섎�濡�,
    // �ш린�쒕뒗 �밸퀎�� �쇱쓣 �� �섎뜑�쇰룄 ��젣�섎㈃ �� �⑸땲��.
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartEthernetTask */
/**
* @brief Function implementing the EthernetTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEthernetTask */
void StartEthernetTask(void *argument)
{
  /* USER CODE BEGIN StartEthernetTask */

  // 1. LwIP 珥덇린�� ��湲� (3珥�)
  osDelay(3000);

  int sock = -1;
  struct sockaddr_in dest_addr;

  // 2. �뚯폆 �앹꽦 �쒕룄 (�깃났�� �뚭퉴吏� 臾댄븳 諛섎났)
  while (sock < 0) {
      sock = socket(AF_INET, SOCK_DGRAM, 0);

      if (sock < 0) {
          // �ㅽ뙣 �� �뚮�遺� 耳쒖쭚
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
          osDelay(500);
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
          osDelay(500);
      } else {
          // �깃났 �� �뚮�遺� �꾧린
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      }
  }

  // 3. 紐⑹쟻吏� �ㅼ젙 (�쇱쫰踰좊━ �뚯씠 IP)
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(8080);
  dest_addr.sin_addr.s_addr = inet_addr("10.10.14.82");

  CanQueueItem recv_item;
  UdpPacket packet;

  for(;;)
  {
      // 4. �� ��湲� (1珥� ���꾩븘��)
      osStatus_t status = osMessageQueueGet(myQueue01Handle, &recv_item, NULL, 1000);

      if(status == osOK) {
          // [�뺤긽 �섏떊]
		  // [정상 수신]
    	  switch (recv_item.id)
    	  {
    	  	  case 0x201: val_ultrasonic = recv_item.data[0]; break; // 초음파
    	  	  case 0x202: val_motor = recv_item.data[0]; break;       // 모터

    	  	  /*case 0x104: val_temp = recv_item.data[0]; break;       // 온도
			  case 0x105: val_humi = recv_item.data[0]; break;       // 습도
			  case 0x201: val_button = recv_item.data[0]; break;     // 버튼
			  case 0x202: val_led = recv_item.data[0]; break;        // LED*/

			  case 0x101: // 통합 데이터 (Board B/D에서 묶어 보낸 경우)
				  val_temp = recv_item.data[0];
				  val_humi = recv_item.data[1];
				  val_led = recv_item.data[2];
				  break;
    	  }

          packet.can_id = recv_item.id;
          memcpy(packet.data, recv_item.data, 8);
      } else {
          // [���꾩븘��] �뚯뒪�몄슜 �붾� �곗씠��
          packet.can_id = 0x999;
          memset(packet.data, 0, 8);
          packet.data[0] = 77;
      }

      // 5. �⑦궥 �ъ옣
      packet.start_byte = 0xAA;
      packet.dlc = 8;

      // 6. UDP �꾩넚
      int sent_bytes = sendto(sock, (char*)&packet, sizeof(packet), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

      if (sent_bytes > 0) {
          // �깃났: 珥덈줉遺� 源쒕묀��
          HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
      } else {
          // �ㅽ뙣: �뚮�遺� 耳쒖쭚 (�뚯폆 �딄� �ъ젒�� �쒕룄)
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
          close(sock);
          sock = -1;
          while(sock < 0) {
              sock = socket(AF_INET, SOCK_DGRAM, 0);
              osDelay(1000);
          }
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      }
  }
  /* USER CODE END StartEthernetTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
