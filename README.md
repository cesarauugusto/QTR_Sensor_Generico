# sensor_csr

Biblioteca **sensor_csr** para Arduino/C++, desenvolvida por  
**César Augusto Victor** — Mestrando em Engenharia Elétrica e de Computação (UFC Sobral)  
📧 E-mail: [cesartri2012@gmail.com](mailto:cesartri2012@gmail.com)

Licença: [MIT](LICENSE)

---

## 📘 Visão Geral

A biblioteca **sensor_csr** foi criada para uso com **barras de sensores QTR genéricos** (não-Pololu), amplamente utilizadas em robôs seguidores de linha.  
Ela implementa uma interface simplificada, compatível com a estrutura da biblioteca **QTRSensors** da Pololu, mas adaptada para sensores analógicos de baixo custo e sem controle de emissor IR.
![Demonstração do sensor_csr](docs/sensor.jpg)

Permite:
- Calibração individual dos sensores.
- Leitura analógica e normalização (0–1000).
- Geração automática de um **padrão binário** (0 = branco, 1 = preto).
- Cálculo do **erro discreto da linha** em intervalos de `-4000` a `+4000`.
- Função direta `ErroSensor()` para integração com o código do robô.

---

## ⚙️ Como funciona

Cada sensor da barra lê um valor **analógico** (0–1023) proporcional à luz refletida pela superfície:
- **Superfícies brancas** → refletem mais luz → valores altos.
- **Superfícies pretas** → refletem menos luz → valores baixos.

Durante a calibração (`qtr.calibrate()`), a biblioteca coleta os valores **mínimos e máximos** que cada sensor pode medir, e usa isso para normalizar a saída na escala **0 a 1000**.

Depois disso, a leitura de cada sensor pode ser comparada com dois **limiares definidos no seu código**:

#define LIMIAR_BRANCO 600
#define LIMIAR_PRETO  900

- Valores abaixo de LIMIAR_BRANCO → considerados branco (0)

- Valores acima de LIMIAR_PRETO → considerados preto (1)

- Valores intermediários → zona “cinza” → tratados como 0

- Esses limiares variam conforme:

- O tipo do sensor (QTR genérico, TCRT5000, etc.),

- O tipo de pista (fita preta em fundo branco ou o inverso),

- A iluminação do ambiente.

🧪 Como determinar o limiar ideal

Monte seu robô sobre a pista.

No código de teste, use algo como:

uint16_t valores[NUM_SENSORES];
qtr.readRaw(valores);
for (int i = 0; i < NUM_SENSORES; i++) {
  Serial.print(valores[i]);
  Serial.print("\t");
}
Serial.println();
delay(200);


Leia os valores no Serial Monitor:

Coloque o sensor sobre o branco da pista → anote a média.

Coloque sobre o preto da linha → anote a média.

Defina os limiares no .ino:

#define LIMIAR_BRANCO  valor_médio_branco
#define LIMIAR_PRETO   valor_médio_preto


Exemplo:

#define LIMIAR_BRANCO 600
#define LIMIAR_PRETO  900


Esses valores são passados à biblioteca toda vez que ErroSensor() é chamada.

🧠 Funcionamento interno simplificado

Leitura analógica:
Cada sensor é lido várias vezes (média definida por setSamplesPerSensor).

Normalização:
Cada valor é mapeado para 0–1000 conforme os limites calibrados.

Binarização:
Cada sensor é classificado como 0 (branco) ou 1 (preto) com base nos limiares definidos.

Cálculo do erro:
O centro da linha é calculado considerando a média ponderada dos sensores ativos ('1').
O erro resultante é um múltiplo de 1000:

-4000 → linha à esquerda

0 → centrado

+4000 → linha à direita

Saída:
A função ErroSensor() retorna esse erro e preenche um array bits[] que mostra o padrão lido (ex.: "00011000").


