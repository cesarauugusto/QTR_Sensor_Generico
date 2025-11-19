# 🚀 sensor_csr
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Arduino](https://img.shields.io/badge/Arduino-Compatible-blue.svg)](https://www.arduino.cc/)
[![C++](https://img.shields.io/badge/Language-C++-brightgreen.svg)](https://isocpp.org/)

Biblioteca para sensores **QTR genéricos (não-Pololu)** desenvolvida por  
**César Augusto Victor**, Mestrando em Engenharia Elétrica e de Computação — UFC Sobral  
📧 cesartri2012@gmail.com  
📘 **DOI:** https://doi.org/10.5281/zenodo.17593098
---
Bilioteca aplicada ao Robo seguidor de Linha do Repositorio:**
https://github.com/cesarauugusto/Projeto_Robo_Seguidor
<p align="center">
  <img src="docs/seguidorgif.gif" width="200">
</p>

## 🧩 Sobre a Biblioteca

A **sensor_csr** fornece uma interface simples e robusta para barras de sensores **QTR genéricos analógicos**, amplamente utilizadas em **robôs seguidores de linha**.
<p align="center">
  <img src="docs/sensor.jpg" width="450">
</p
Ela segue a base da QTRSensors original da Pololu, mas adiciona:

- 📌 Calibração simplificada  
- 📌 Cálculo de erro discreto padrão de robótica (múltiplos de 1000)  
- 📌 Sistema nativo de **detecção de GAP (pistas tracejadas)**  
- 📌 Função única **ErroSensor()**, que retorna automaticamente:

bits → padrão lido (ex.: "00011000")
erro → deslocamento da linha (ex.: -2000, 0, +4000)
gap → detecta traçados tracejados

---

## ⚙️ Recursos da Biblioteca

✔ Calibração automática  
✔ Normalização das leituras (0–1000)  
✔ Conversão binária (0 = branco / 1 = preto)  
✔ Erro discreto de -7000 a +7000  
✔ Função `detectarGAP()` integrada  
✔ Cálculo interno do último erro válido (`ultimoErroValido`)  
✔ Compatibilidade com DRV8833, L298N e TB6612FNG  
✔ Exemplo completo de robô incluso

---

## 🧠 Nova Função: GAP Detection (Tracejado)

A biblioteca agora inclui a função:

```cpp
qtr.gapDetection()
```
Ela identifica automaticamente quando a barra lê:
00000000
Quando isso ocorre, significa que o robô:
entrou em um trecho tracejado ou saiu momentaneamente da linha por irregularidade da pista.

🔧 Comportamento:
O código detecta o GAP.
A biblioteca retorna true.
O robô passa a usar o último erro válido:

```cpp
bool gap = qtr.gapDetection();
if (gap)
    erro = ultimoErroValido;
else
    ultimoErroValido = erro;
```

Assim que um sensor voltar a enxergar 1, o GAP encerra.
Isso permite ao robô atravessar tracejados sem oscilações e sem perder a linha.

📂 Exemplos Incluídos
exemple - Teste exemplo para verificação dos sensores
codigo_robo	- Controle completo com GAP e PID.

📌 Recomendação do uso no Robô Seguidor de Linha.
1️⃣ Abra o exemplo codigo_do_robo
2️⃣ No código, configure:

```cpp
#define SENSOR_DEBUG 1
```
Isso fará o robô não movimentar os motores e apenas imprimir os valores dos sensores cruamente.

🔍 1. Medindo o valor da linha preta
Coloque todos os sensores exatamente sobre a linha preta.
No Serial Monitor você verá valores como:

Copiar código
850   870   900   910   ...
→ Anote a média.

🔍 2. Medindo o valor do fundo branco
Coloque todos os sensores na área branca da pista:

Copiar código
300   350   420   380   ...
→ Anote a média.

🎯 3. Definindo os limiares finais
Use ESTE critério:

Medida	Exemplo	Limiar recomendado
Branco medido	400	coloque 500
Preto medido	900	coloque 800

Ou seja:

```cpp
#define LIMIAR_BRANCO 500
#define LIMIAR_PRETO  800
```
Isso vai filtrar ruídos e garantir a leitura estável.
Após o ajuste dos Sensores altere SENSOR_DEBUG Para 0

```cpp
#define SENSOR_DEBUG 0
```

🔄 Ajustando a Direção do Controle (TURN_SIGN)
Se durante o teste o robô:

virar para o lado ERRADO, ou reagir ao erro invertido, basta trocar TURN_SIGN:

```cpp
#define TURN_SIGN +1
```
👉 Ou:
```cpp
#define TURN_SIGN -1
```
Teste na prática em uma curva para garantir o sentido correto.

# 🤖 Como o Erro é Calculado
A função:
```cpp
int erro = qtr.ErroSensor(bits, LIMIAR_BRANCO, LIMIAR_PRETO, false, 0);
```
retorna valores como:
-7000  -6000 ... -1000   0   +1000 ... +7000
Isso representa:

negativo → linha à esquerda

zero → centralizado

positivo → linha à direita

📊 Tabela Simplificada de Erro
Cada padrão binário corresponde a um erro discreto configurado na biblioteca.

Ex.:
00011000  → erro = 0   (centralizado)
00111000  → erro = -1 * 1000
00001110  → erro = +3 * 1000
10000000  → erro = +7 * 1000

🧾 Citação
Se utilizar esta biblioteca em projetos acadêmicos:

César Augusto Victor. (2025). Library for generic QTR sensors (1.0). Zenodo.
https://doi.org/10.5281/zenodo.17593098

📜 Licença
Licenciado sob MIT License — livre para uso pessoal, acadêmico e comercial, desde que citada a autoria.
© 2025 César Augusto Victor — Universidade Federal do Ceará (UFC - Sobral)

⭐ Se este projeto te ajudou, deixe uma estrela no repositório!



