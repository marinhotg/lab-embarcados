# Ensaios de host

Conferem, sem placa, as duas partes do firmware que sao matematica pura e nao
dependem de hardware:

- `test_odom` - cinematica diferencial (equacoes 1 a 6 da secao 4.5)
- `test_payload` - serializacao JSON da telemetria (secao 4.6)

Os testes nao duplicam codigo: `run_tests.sh` recorta os trechos relevantes de
`main/odometry.c` e `main/telemetry.c` e compila com stubs de host. Se alguem
mexer na formula ou no payload, o ensaio acompanha.

    ./run_tests.sh
