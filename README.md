# makelight

* D8 - Power on/off als Verstärker (3.3V)
* D6 - Power on/off lichtsensor (Senke)
* RX - Data LEDs WS2812
* A0 - Lichtwiderstand (+>-|=L1=|-A0-|=Poti=|-D6)

Schaltet ein paar LEDs und liest einen lichtabhängigen Widerstand an A0 aus.

Kann via MQTT angesprochen werden.

# todo
- implementiere mal eine Schnittstelle die eine Info über das Gerät zurück liefert via MQTT. /flaschengeist/info oder so
