# El Profesor - Modulador de voz

Projeto de modulador de voz utilizando microcontrolador ARM STM32 Cortex-M3, vulgo "BluePill"

INTEGRANTES

  Ítalo Viana Severo   
  João Luca Ripardo Teixeira Costa   
  Jonas Albuquerque Araújo


MOTIVAÇÃO

Desde um efeito engraçado no áudio de um vídeo até um disfarce para preservar a identidade de partes em uma conversa, a modulação de voz gera transformações interessantes e convenientes em um determinado sinal. A modulação na voz pode funcionar como uma camada extra na segurança de um canal de comunicação, mais especificamente quando existe o risco de um dos interlocutores ser identificado pelo timbre de sua voz. Talvez o exemplo mais claro seja o de testemunhas de um crime, que utilizam de tal modificação em sua voz para que os criminosos não tenham acesso a sua identidade. Dessa forma, viemos com a proposta de um sistema com a capacidade de captar e transmitir a voz modulada em tempo real.


DESCRIÇÃO

Inicialmente, o sistema será composto por dois módulos: um módulo gravador/emissor, responsável por captar, modular e enviar o sinal da voz, e um módulo receptor/reprodutor, responsável por receber o sinal da voz e reproduzir. O primeiro módulo é composto por um circuito com microfone, que captará as amostras do áudio, o microcontrolador, responsável por fazer a leitura e a modificação do sinal, e o emissor RF 433 MHz, responsável por transmitir o sinal para o outro módulo. Além disso, um potenciômetro funcionará como chave para deixar a voz mais ou menos grave. O segundo módulo será composto por receptor RF 433 MHz, um microcontrolador para fazer a devida manipulação do sinal, e um alto falante que irá reproduzir o som. 

