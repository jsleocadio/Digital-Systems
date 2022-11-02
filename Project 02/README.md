# Projeto U2

**Aluno:** Jefferson dos Santos Leocadio

[**Link Tinkercad**](https://www.tinkercad.com/things/8DLrgrBghb2-copy-of-projeto-u2/editel?sharecode=3rmyxzhpChyPTEcj4WAeO7WUaZ97B0x2VQAHetrqyY8)

*****

Seu grupo foi contratado para desenvolver o computador de bordo de um novo
veículo utilizando o microcontrolador ATMega328p. As especificações do sistema a ser desenvolvido são encontradas abaixo:

O sistema é composto por três sinais analógicos como entrada para os conversores
AD, quatro sinais de entrada digital, quatro sinais de saídas digitais e um de saída analógica
(PWM).

![Configuração do Projeto!](https://github.com/jsleocadio/Digital-Systems/blob/main/Project%2002/images/Imagem%2001.png)

**Materiais utilizados:**

7 - Resistores 10kΩ;

2 - potenciômetros;

1 - ATMega328p;

4 - Push Button;

4 - LEDS;

4 - Resistores de 220Ω;

1 - Servo Motor.

**Ligação dos Componentes:**
* Os sinais de Ignição, sensor de velocidade e sensor de combustível devem estar
conectados nas entradas analógicas AD0, AD1 e AD2 respectivamente.
* Os botões B1, B2, B3 e B4 devem ser conectados aos pinos digitais PB2, PB3, PB4
e PB5 respectivamente.
* Os sinais de ligar pisca D, pisca E, Injeção Eletrônica e Faróis devem ser
conectados nos pinos PD4, PD5, PD6 e PD7 respectivamente.
* O sinal de ligar limpador de vidros deve estar conectado no pino PB5
* O Servo Motor (Limpador de parabrisa) deve estar ligado na saída analógica PB1;

**Funcionamento:**

O funcionamento do sistema é controlado pela chave de ignição, existem três
estados possíveis:
1. **Desligado:** Todos os elementos do sistema estão desligados. Todo o sistema deve
estar esperando alguma informação do ADC0
2. **Ligar Faróis:** Nesse segundo nível do valor de ADC0, o sistema só poderá alterar o
estado dos faróis e piscas, bem como ler o nível de combustível;
3. **Ligar motor:** Led de injeção eletrônica acende, todos os componentes do sistema
funcionam, incluindo piscas, faróis e limpador de parabrisas. Além disso, deve
receber sinais vindo dos sensores de velocidade e nível de combustível.

No estado 2, apenas o nível do tanque deve ser apresentado no monitor serial,
enquanto no nível 3 o monitor serial deve apresentar o nível de combustível e a velocidade.

**Informações adicionais:**
* O potenciômetro conectado ao ADC2 representa o sensor de velocidade do sistema;
* O potenciômetro conectado ao ADC1 representa o sensor de nível de combustível;
* Os sinais dos piscas devem funcionar assim como nos veículos. Considere colocar
um intervalo entre 500ms a 1s em cada um.
* As informações referentes a velocidade e nível de combustível devem ser mostradas
no monitor serial a cada 500ms.
* Lembre-se que o sistema tenta simular um sistema real, portanto mapeie os valores
das entradas analógicas e PWM para valores encontrados em veículos comerciais.
* Para controle de tempo, use a função [millis()](https://www.arduino.cc/reference/pt/language/functions/time/millis/).
* O sistema a ser embarcado **DEVE SER ESCRITO EM C (AVR)**.

***Quaisquer adições para melhor funcionamento do sistema serão considerados.

**Entregar relatório sobre o desenvolvimento do projeto e o link do sistema
funcionando na plataforma do Tinkercad.**

# Montagem

## Entrada Analógicas

A ignição é controlada por três resistores de 10 k $\Omega$, ou seja, um divisor de tensão. Na protoboard, realizei a configuração da seguinte forma: 

![Configuração Divisor de Tensão!](https://github.com/jsleocadio/Digital-Systems/blob/main/Project%2002/images/Imagem%2002.png)

**Adicionalmente:** Para simular uma chave de ignição adicionei um potenciômetro de 1 G $\Omega$ conforme imagem complementar abaixo:

![Potenciômetro servindo como chave de ignição!](https://github.com/jsleocadio/Digital-Systems/blob/main/Project%2002/images/Imagem%2003.png)

Utilizei 2 potenciômetros de 10 k $\Omega$ para servir como medidores de velocidade e combustível. Montados da seguinte forma:

![Montagem Entradas Analógicas!](https://github.com/jsleocadio/Digital-Systems/blob/main/Project%2002/images/Imagem%2004.png)

Conforme solicitado as entradas analógicas foram configuradas da seguinte forma:

|Entrada|Conectado à|
|---|---|
|AD0|Chave de ignição|
|AD1|Velocímetro|
|AD2|Medidor de combustível|

## Entradas Digitais

As entradas digitais serão utilizadas para monitorar os botões que serão usados, respectivamente, para ligar o pisca direito, o pisca esquerdo, os faróis e o limpador. A montagem foi feita conforme projeto:

![Ligação dos botões!](https://github.com/jsleocadio/Digital-Systems/blob/main/Project%2002/images/Imagem%2005.png)

**Lembrando que resitores de 10k $\Omega$ são utilizado na ligação dos botões.

Segue tabela de ligações:

|Entrada|Conectada à|Liga|
|---|---|---|
|PB2|Botão 1|Pisca direita|
|PB3|Botão 2|Pisca esquerdo|
|PB4|Botão 3|Faróis|
|PB5|Botão 4|Limpador Para-brisa|

## Saídas Digitais

As saídas digitais serão utilizadas para ligar, respectivamente, pisca direita, pisca esquerda, injeção eletrônica e faróis. A montagem foi executada da seguinte forma:

![Montagem entradas e saídas digitais!](https://github.com/jsleocadio/Digital-Systems/blob/main/Project%2002/images/Imagem%2006.png)

**Lembrando que para utilizar a tensão correta na ligação dos LEDs é necessário utilizar resistores de 220 $\Omega$.

As conexões foram feitas da seguinte forma:

|Saída|Liga|
|---|---|
|PD4|Pisca direito|
|PD5|Pisca esquerdo|
|PD6|Injeção eletrônica|
|PD7|Faróis|

## Saída Analógica (PWM)

Precisamos usar um PWM (Pulse Width Modulation) para controlar o servo motor, pois, segundo o datasheet do SG90 precisa de um pulso específico.

![Datasheet do SG90!](https://github.com/jsleocadio/Digital-Systems/blob/main/Project%2002/images/Imagem%2007.png)

Como visto na imagem acima, precisamos de um pulso de 50 Hz com um duty cicle de 1 à 2 ms. Falarei mais sobre isso na parte de *coding*.

A montagem foi feito da seguinte forma:

![Ligação saída analógica!](https://github.com/jsleocadio/Digital-Systems/blob/main/Project%2002/images/Imagem%2008.png)

Fazendo com que a porta PB1 seja a saída que regulará a rotação do limpador de para-brisa.

## Considerações de montagem

Conforme a imagem abaixo, a montagem foi realizada conforme projeto:

![Montagem final do projeto!](https://github.com/jsleocadio/Digital-Systems/blob/main/Project%2002/images/Imagem%2009.png)

A montagem ocorreu sem grandes dificuldades. Agora iremos para o *coding*.

# *Coding* (Codificando)

Nesta seção abordaremos a codificação do projeto passo-a-passo:

## Variavéis globais

Para funcionamento do sistema declarei as seguintes váriaveis como globais:

```
const unsigned int TOP = 4999;
const unsigned int TMINUS90 = 249; // TMINUS90 is the value where the servo motor turns -90º, to the left.
const unsigned int T90 = 499; // T90 is the value where the servo motor turns 90º, to the right.
int ledRight = LOW;
int ledLeft = LOW;
int headLights = LOW;
int wiper = LOW;
int ignitionState = 0;
int fuel = 0;
int speed = 0;
unsigned long start = 0;
```

## DDRs e PORTs

Iniciamos declarando quais serão as saídas do sistema. Para isso, na declação da DDR colocamos, na posição do pino correspondente, em nível alto. Criei uma função conforme código abaixo:

```
//Set Ports to be used
void DDRBegin()
{
  DDRB |= 0b00000010;	//Set PORTB2 as OUTPUT
  PORTB &= 0;
  DDRD |= 0b11110000;	//Set PORTD4|PORTD5|PORTD6|PORTD7 as OUTPUT
  PORTD &= 0;
}
```

## Entradas Analógicas

Para fazermos a conversão de valores analógicos para digitais usaremos o ADC do ATMega. Começamos iniciando os registradores da seguinte forma:

```
//Set Analogic inputs and ADC
void ADCBegin()
{
  ADMUX |= (1<<REFS0);
  ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
}
```

No registrador `ADMUX` colocamos em nível alto o pino `REFS0`. Fazendo isso, utilizaremos a tensão de referência do próprio ATMega que é de 5V.

No registrador `ADCSRA` colocamos em nível alto o pino `ADEN` que habilita o conversor analógico-digital. Os pinos `ADPS0`, `ADPS1` e `ADPS2` são o pre-scale, colocando todos em nível alto indica que dividiremos o clock do ATMega por um fator de 128.

### Chave de Ignição - AD0

Teremos três entradas analógicas, a primeira será a ignição, que fica na entrada `AD0`. Para lermos o valor da ignição criamos a função `getVoltage()`:

```
//Calculate voltage on ignition
void getVoltage(){
  ADMUX &= 0b11110000;
  ADMUX |= 0b00000000;
  ADCSRA |= (1<<ADSC);
  while(!(ADCSRA & (1<<ADIF)));
  ignitionState = ADC;
}
```

Na primeira instrução da função dizemos ao `ADMUX` as configurações da conversão. Os dois bits mais significativos carregam as configurações da tensão de referência carregadas previamente, no terceiro bit mais significativo, o `ADMUX` se prepara para passar os dados para o registrador de dados `ADC`.

Na segunda instrução, selecionamos a porta analógica. Ao manter todos os pinos em nível baixo, escolheremos o pino `AD0`.

Na terceira instrução dizemos ao registrador `ADCSRA` para iniciar a conversão. 

Utilizamos um `while` para carregar as informações no `ADC` a condição de parada deste *loop* é o `ADIF` que é um interruptor do conversor que indica quando o registrador `ADC` recebeu todas as informações.

Aqui utilizamos nossa variavél global `ignitionState` para salvar o valor salvo em `ADC`.

### Medidor de Combustível - AD1

Para medirmos o nível de combustível foi criado a função `printFuel`:

```
//Print fuel gauge
void printFuel(){
  ADMUX &= 0b11110000;
  ADMUX |= 0b00000001;
  ADCSRA |= (1<<ADSC);
  while(!(ADCSRA & (1<<ADIF)));
  fuel = float(ADC)/1023 * 100;
  Serial.print("Fuel: ");
  Serial.print(fuel);
  Serial.println(" %.");
}
```

A principal diferença para a entrada anterior é que colocaremos, na segunda instrução, o bit menos significativo em nível alto. Com isto, estamos informando para utilizar o pino `AD1`.

Após guardar o valor em `ADC`. Fazemos uma conversão para que o valor esteja em percentual, com isto, dividimos o valor de `ADC` por 1023, que é o valor máximo armazenado em `ADC` e multiplicamos por 100.

Após isso, usamos das funções `Serial.print()` e `Serial.println()` para mostrar o valor obtido.

### Velocímetro - AD2

Para medirmos a velocidade foi criado a função `printSpeed()`:

```
//Get Vehicle Speed
void printSpeed(){
  ADMUX &= 0b11110000;
  ADMUX |= 0b00000010;
  ADCSRA |= (1<<ADSC);
  while(!(ADCSRA & (1<<ADIF)));
  speed = float(ADC)/1023 * 120;
  Serial.print("Speedmeter: ");
  Serial.print(speed);
  Serial.println(" Km/h.");
}
```

Mais uma vez, a principal diferença é na segunda instrução, aonde colocamos o segundo bit menos significativo em nível alto para lermos o valor de `AD2`.

A nível de simulação, usamos um automóvel 1.0 com velocidade máxima de 120 Km/h. Adotando este valor, faremos a divisão do valor em `ADC` por 1023 e multiplicaremos por 120.

Após isso, mostramos no Serial o valor obtido, de forma semelhante ao do combustível.

## Entradas Digitais

Conforme nosso projeto, as entradas digitais são nossos *push buttons*.

Então para verificar se eles foram apertados, utilizarei de uma estrutura condicional.

```
//Verify pressed buttons:
    if (PINB & 1<<2){
      ledRight = !ledRight;
    }
    if (PINB & 1<<3){
      ledLeft = !ledLeft;
    }
    if (PINB & 1<<4){
      headLights = !headLights;
    }
    if (PINB & 1<<5){
      wiper = !wiper;
    }
```

Nas instruções acima, que estão dentro do `void loop()`, estamos definindo que quando o botão for pressionado, ele atualizará o valor da variavél global ligada ao botão para o negado do seu valor atual. Exemplo: todas as váriaveis declaradas nos botões são iniciadas em `LOW`, ao pressionar o botão, seu valor muda para `HIGH`. Será necessário armazenar estes valores afim de não precisar manter o botão pressionado para que a função atribuída a ele seja realizada.

## Saídas Digitais

As saídas digitais serão utilizadas pelos LEDs. 

No passo anterior armazenamos os valores dos botões pressionados em variavéis e agora vamos ler seu estado para saber se haverá saída ou não:

```
//Set Right LED to blink for 0.5s
void blinkRightLed(){
  if (ledRight){
    PORTD |= 1<<4;
    _delay_ms(500);
    PORTD &= ~(1<<4);
    _delay_ms(450);
  }
}

//Set Left LED to blink for 0.5s
void blinkLeftLed(){
  if (ledLeft){
    PORTD |= 1<<5;
    _delay_ms(500);
    PORTD &= ~(1<<5);
    _delay_ms(450);
  }
}

//Turn on the Eletronic Injection LED
void turnOnEletronicInjection(){
  PORTD |= 1<<6;
}

//Turn on the Headlights
void turnOnHeadLights(){
  if (headLights) {
    PORTD |= 1<<7;
  }
}
```

Nas primeiras funções, `blinkRightLed()` e `blinkLeftLed()`, verificamos se as variavéis `ledRight` e `ledLeft` estão em nível alto, caso positivo, seus LEDs ficarão ativos por 500 ms e desativos por 500 ms.

A injeção eletrônica estará sempre ativa quando a ignição estiver no nível máximo dele, portanto não há necessidade de guardar nenhuma variavél.

Para os faróis, o LED ficará ativo até que o botão seja apertado novamente. Portanto, fazemos a verificação se o botão foi pressionado e manteremos ativo até que o botão seja pressionado mais uma vez.

## Saída Analógica (PWM)

Para acionamento do servo motor, como foi visto no *datasheet* é necessário um sinal retangular de 50 Hz, onde temos um *duty cicle* variando entre 1 e 2 ms.

Inicialmente para configurar a frequência do sinal PWM, precisamos calcular o TOP, visto que a frequência já temos.

O cálculo é feito da seguinte forma:

$$TOP = \frac{\omega_{clk}}{N*\omega_{desired}} - 1,$$

onde: 
* $\omega_{clk} \rightarrow$ frequência do ATMega (16 MHz);
* $N \rightarrow$ Valor do *pre-scale*;
* $\omega_{desired} \rightarrow$ Valor da frequência desejada, nosso caso 50 Hz.

Dito isto, inicializamos nosso PWM da seguinte forma:

```
void PWMBegin()
{
  // Stop Timer/Counter1
  TCCR1A = 0;  // Timer/Counter1 Control Register A
  TCCR1B = 0;  // Timer/Counter1 Control Register B
  
  ICR1 = TOP;
  
  // Set clock prescale to 64
  TCCR1B |= (1 << CS11) | (1 << CS10); //pre-scaler 64

  // Set to Timer/Counter1 to Waveform Generation Mode 14: Fast PWM with TOP set by ICR1
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12) ;
}
```

Primeiro, iniciamos nosso *pre-scale* com o valor de 64, colocando os pinos `CS11` e `CS10` em nível alto. E informamos que a contagem máxima para um pulso de 50 Hz, segundo o cálculo mostrado acima é de 4999. Este valor foi atribuído a variável `TOP`, que será recebida pelo registrador `ICR1`.

Configuramos para que o sinal seja um *Fast PWM*.

Após isto, configuramos a porta:

```
void PWMEnable1A()
{
  // Enable Fast PWM on Pin 9
  TCCR1A |= (1 << COM1A1);
}
```

Agora iremos criar uma função para definir o *duty cicle*. Ela receberá um valor que será adicionado ao comparador `OCR1A`.

```
void PWM1A(unsigned int PWMValue)
{
  OCR1A = PWMValue;
}
```

O comparador `OCR1A` será utilizado para contar até o valor `PWMValue`, vale salientar que durante a contagem, o PWM estará em nível alto, quando a contagem passar do valor informado ao `OCR1A` até chegar ao `TOP`, ficará em nível baixo.

Precisamos calcular agora o *duty cicle* necessário para que o servo motor funcione semelhante ao limpador de para-brisa. Para isso, o servo motor da posição inicial precisa andar 90° para a esquerda, sentido anti-horário, e depois -90°, ou virar para a direita, no sentido horário. 

Segundo o *datasheet* do servo motor SG90, precisamos de um *duty cicle* de 1 ms para que o servo motor vire 90° para a esquerda. Já calculamos que se o ATMega contar de 0 até 4999, levará 20 ms. Então para chegar encontrar o valor para que ele conte apenas 1 ms:

$$T90 = \frac{TOP}{20} = \frac{4999}{20} = 249$$

O *duty cicle* de 2 ms será necessário para que o servo motor rotacione -90°:

$$TMINUS90 = \frac{2*TOP}{20} = \frac{4999}{10} = 499$$

Se notarmos nas variavéis globais, estes valores já estão lá.

Com isto, basta criar uma função em que uma vez que apertem o botão, o servo motor fará o movimento de 90° e após um tempo o movimento de -90°:

```
void turnOnWiper(){
  if (wiper){
    PWM1A(T90);
    _delay_ms(600);
    PWM1A(TMINUS90);
    _delay_ms(500);
  }
}
```

## Inicialização

Em nosso `void setup()` iremos colocar todas as funções de inicialização já comentadas até aqui:

```
void setup(){
  Serial.begin(9600);
  DDRBegin();			//Call Ports
  ADCBegin();			//Call ADC
  PWMBegin();			//Call PWM
  
  PWM1A(0);
  PWMEnable1A();
}
```

## Funcionamento do Sistema

Em nosso `void loop()` iniciamos nos certificando que todas os pinos estão em nível baixo. Após isso, verificamos a voltagem na ignição, visto que há três estágios a serem observados:

|Estado|Tensão|Valor em `ignitionState`|
|---|---|---|
|$0$|$0.00 V$|$x < 341$|
|$1$|$1.67 V$|$341 \leq x < 682$|
|$2$|$3.33 V$|$682$|

Estes valores em `ignitionState` será atualizado pela função `getVoltage()`. Após isto, verificamos se algum botão foi pressionado. E em seguida verificamos em qual estado está e o que é permitido que ele faça.

A implementação:

```
void loop(){
   //Make sure that all Outputs are LOW
    PORTD &= 0;
    
    //Get Voltage
    getVoltage();
    
    //Verify pressed buttons:
    if (PINB & 1<<2){
      ledRight = !ledRight;
    }
    if (PINB & 1<<3){
      ledLeft = !ledLeft;
    }
    if (PINB & 1<<4){
      headLights = !headLights;
    }
    if (PINB & 1<<5){
      wiper = !wiper;
    }
    //Last Stage of ignition: Can turn on everything
    if (ignitionState == 682){
      turnOnEletronicInjection();
      blinkRightLed();
      blinkLeftLed();
      turnOnHeadLights();
      turnOnWiper();
      unsigned long end = millis();
      if (end - start >= 500) {
        start = end;
        printFuel();
        printSpeed();
      }
    } 
    // Second stage of ignition: Can turn something
    else if (ignitionState >= 341){
      blinkRightLed();
      blinkLeftLed();
      turnOnHeadLights();
      unsigned long end = millis();
      if (end - start >= 500) {
        start = end;
        printFuel();
      }
    }
  _delay_ms(50);
}
```

Conforme solicitado no projeto, o display Serial só deve mostrar as informações a cada 500 ms. Por isso um comparador de tempo para saber a cada iteração se já passou o tempo requerido para mostrar as informações.

# Considerações finais

Houve uma tentativa de realizar a codificando usando explicitamente C/AVR, porém, houve dificuldades com a função `millis()` que só retornava 0 quando rodava dentro da `int main()`. Após migrar para `void setup()` e `void loop()` o problema foi sanado.

Há algumas falhas como encontrar o timing correto para acionar os botões visto que existe apenas um momento muito curto para acioná-los.

No mais tudo funcionando conforme solicitado no projeto.
