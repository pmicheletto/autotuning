1.  Wrapper
    Tentei a fmilib mas teve muito erro com #includes bagunçados dentro da própria biblioteca, tentei a cpp-fmus mas descobri que ela é apenas para geração do FMU. 
    Por fim, a que funcionou foi a FMI4cpp, toda em c++, e pro primeiro teste copiei o modelo "PID_Controller.fmu" e fiz a leitura do sensor de velocidade com speedSensor.w

2. Modelo simulação
    Vou começar com um modelo simples para ver como os dados fluem, um step e uma função de transferencia.

3. Desvio padrão
    O código já esta funcional para usar o desvio padrão para reconhecer comportamentos da dinamica