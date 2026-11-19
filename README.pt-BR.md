*[Read in English](README.md)*

# ESP32-HAM-CLOCK-ptbr

Fork do [SP3KON/ESP32-HAM-CLOCK](https://github.com/SP3KON/ESP32-HAM-CLOCK) com **tradução para Português (Brasil)** e **tamanho de tabela MEDIUM**.

## O que tem de novo neste fork

### Idioma Português (Brasil)
- Novo `TFT_LANG_PT` com tradução completa para PT
- Terceira opção de idioma no menu TFT (PL/EN/PT)
- Strings de data em português, telas de clima/alerta, info de bandas HF
- Requisições da API OpenWeather usam `lang=pt_br`
- Interface web: `indexPT.html`, `manual_pt.txt` com rotas PT
- Tratamento de acentos (converte Latin-1 para ASCII no display)

### Tamanho de tabela MEDIUM
- Novo modo de layout entre SMALL e LARGE
- Usa Font 2 embutida (16px), pitch de 20px, 9 linhas
- Funciona com telas DX, POTA, HamAlert, APRS-IS
- Opção web e suporte NVS/backup (`table_size=medium`)

## Projeto original

Este é um relógio HAM e terminal DX/APRS para ESP32 (principalmente CYD ESP32-2432S028) com display TFT 2.4" e painel web paralelo.

Funcionalidades:
- DX Cluster (telnet)
- POTA (API)
- HAMALERT (telnet)
- APRS-IS (RX + beacon TX opcional)
- Clima e previsão (OpenWeather)
- Dados de propagação/solar

Veja o [repositório original](https://github.com/SP3KON/ESP32-HAM-CLOCK) para documentação completa.

## Branch

A tradução em português e o tamanho MEDIUM estão no branch `feature/pt-br`. Há um PR aberto (#17) para o projeto original.

## Créditos

- Projeto original: [SP3KON](https://github.com/SP3KON/ESP32-HAM-CLOCK)
- Tradução para português e tamanho MEDIUM: PP5RD
