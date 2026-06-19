package si;

import robocode.*;
import robocode.util.Utils;
import java.awt.Color;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * Robotnikka, o Robô deus do Sol vê o futuro dos inimigos e antecipa.
 *
 * Faz a previsao com um algoritmo de predicao baseado em KNN e arvores
 * K-dimensionais para atirar onde o alvo estara futuramente.
 */
public class Robotnikk extends AdvancedRobot {
    private double velocidadeInimigo;
    private double ultimaVelocidadeInimigo; //vel do inimigo no tick anterior
    private double velocidadeLateralInimigo; //vel perpendicular a visao do robo
    private double tempoUltimaMudancaVelocidade; //quantos tick leva pro inimigo acelerar/desacelerar
    private double tempoUltimaMudancaMinhaVelocidade; //quantos tick o robo leva pra mudar a vel
    private double direcaoMovimento = 1; //sentido horario(1) ou anti(-1), definindo o lado do guessfactor
    private double anguloMaximoFuga; //maior abertura de angulo de fuga do robo de uma bala
    private Point2D.Double minhaPosicao;
    private Point2D.Double posicaoInimigo;
    public double energiaInimigo = 100.0; //usado para monitorar as quedas repentinas (tiro disparado)
    private double distanciaParede; //quao perto os inimigos estao de atingir a parede
    private double distanciaParedeOposta; //inimigos acuados nao conseguem maximizar o guessfactor
    private double minhaDistanciaParede; //para prever onde o inimigo tentara atirar, na situacao acima
    private final double MARGEM_PAREDE = 18; //o centro do robo tem de estar no min 18px distante da parede
    static final double QUASE_METADE_PI = 1.25; //o angulo de surfing (radianos ~ 71 graus)
    private Rectangle2D.Double campoBatalha;
    private double larguraCampo;
    private double alturaCampo;
    private static final double RAIO_CURVA = 91 * 8 / (2 * Math.PI); //circunf min q o robo consegue fazer em vel max sem bater

    private double poderFinalBala;
    private double giroFinalArma;
    //divide o arco de fuga do inimigo em 31 fatias, 15 eh o do meio
    private static final int FATOR_TIRO_CENTRAL = 15;
    private static final int TOTAL_FATORES_TIRO = 31;
    //divide o proprio arco de fuga (mais fatias na fuga que no tiro para dar maior precisao de desvio)
    public static final int FATOR_MOVIMENTO_CENTRAL = 23;
    public static final int TOTAL_FATORES_MOVIMENTO = 47;

    public ArrayList<OndaTiro> ondasTiro; //guarda os tiros do robo, mapeando os pontos em que o inimigo estava quando foi atingido
    public ArrayList<OndaInimiga> ondasInimigas; //usadas para simular seu cruzamento com o robo e este decidir como desviar
    //guardam onde (direcao, angulos) o robo estava ticks atras quando o inimigo atirou uma bala
    public ArrayList<Integer> direcoesSurf;
    public ArrayList<Double> angulosAbsolutosSurf;

    //variaveis de rastreamento para cauterizar a energia do inimigo
    private static int tirosInimigosDestaPartida = 0;
    private static int acertosInimigosDestaPartida = 0;

    //estruturas knn, para calcular semelhança baseado em 6 dimensoes
    public static ArvoreKd.Manhattan<Double> arvoreTiro = new ArvoreKd.Manhattan<>(6, 50000);
    public static ArvoreKd.Manhattan<Double> arvoreSurf = new ArvoreKd.Manhattan<>(6, 50000);
    
    //arrays heuristicos para o cold start, ate a arvore knn tiver condicoes de aprendizado
    public static double[] hibridoTiroFrio = new double[TOTAL_FATORES_TIRO];
    public static double[] hibridoMovimentoFrio = new double[TOTAL_FATORES_MOVIMENTO];


    /**
     * Inicializa as variaveis, configura as cores do robo e inicia o giro continuo do radar.
     */
    public void run() {
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);
        setColors(Color.BLACK, Color.BLUE, Color.BLACK);
        setScanColor(Color.GREEN);

        larguraCampo = getBattleFieldWidth();
        alturaCampo = getBattleFieldHeight();
        campoBatalha = new Rectangle2D.Double(MARGEM_PAREDE, MARGEM_PAREDE, larguraCampo - MARGEM_PAREDE * 2,
                alturaCampo - MARGEM_PAREDE * 2);

        ondasTiro = new ArrayList<>();
        ondasInimigas = new ArrayList<>();
        direcoesSurf = new ArrayList<>();
        angulosAbsolutosSurf = new ArrayList<>();

        do {
            turnRadarRightRadians(Double.POSITIVE_INFINITY);
        } while (true);
    }

    /**
     * Evento disparado ao escanear um inimigo. Atualiza os estados, capta as propriedades e
     * engatilha a fuga da onda e tiro.
     */
    public void onScannedRobot(ScannedRobotEvent e) {
        minhaPosicao = new Point2D.Double(getX(), getY());
        double velocidadeLateral = getVelocity() * Math.sin(e.getBearingRadians());
        double anguloAbsoluto = e.getBearingRadians() + getHeadingRadians();

        direcoesSurf.add(0, (velocidadeLateral >= 0) ? 1 : -1);
        angulosAbsolutosSurf.add(0, anguloAbsoluto + Math.PI);

        if (Math.abs(getVelocity()) - Math.abs(getVelocity() - velocidadeLateral) != 0) {
            tempoUltimaMudancaMinhaVelocidade++;
        } else {
            tempoUltimaMudancaMinhaVelocidade = 0;
        }

        double poderBalaInimiga = energiaInimigo - e.getEnergy();
        if (poderBalaInimiga < 3.01 && poderBalaInimiga > 0.09 && direcoesSurf.size() > 2) {
            tirosInimigosDestaPartida++; // Rastreia o tiro do inimigo para estatisticas

            Robotnikka.OndaInimiga onda = new Robotnikka.OndaInimiga();
            onda.tempoDisparo = getTime() - 1;
            onda.velocidadeBala = Rules.getBulletSpeed(poderBalaInimiga);
            onda.distanciaPercorrida = Rules.getBulletSpeed(poderBalaInimiga);
            onda.direcao = direcoesSurf.get(2);
            onda.anguloDireto = angulosAbsolutosSurf.get(2);
            onda.posicaoDisparo = (Point2D.Double) posicaoInimigo.clone();

            minhaDistanciaParede = calcularDistanciaParede(anguloAbsoluto + Math.PI, e.getDistance(),
                    calcularAnguloMaximoFuga(onda.velocidadeBala, minhaPosicao.distance(onda.posicaoDisparo)));
            double bft = Math.max(1, e.getDistance() / onda.velocidadeBala);
            double velAproximacao = getVelocity() * Math.cos(e.getBearingRadians());

            onda.caracteristicas = new double[] {
                    limitarValor(0, 1, e.getDistance() / 800.0) * 3.0,
                    limitarValor(0, 1, Math.abs(velocidadeLateral) / 8.0) * 4.0,
                    limitarValor(0, 1, Math.abs(getVelocity()) / 8.0),
                    limitarValor(0, 1, (tempoUltimaMudancaMinhaVelocidade / bft) / 1.5) * 2.0,
                    limitarValor(0, 1, minhaDistanciaParede / 1.5) * 2.0,
                    limitarValor(0, 1, (velAproximacao + 8.0) / 16.0)
            };

            ondasInimigas.add(onda);
        }

        energiaInimigo = e.getEnergy();
        posicaoInimigo = projetarCoordenadas(minhaPosicao, anguloAbsoluto, e.getDistance());

        atualizarOndas();
        executarSurfing();

        //poder decai suavemente com a distancia. max: 3.0, min: 0.1
        double poder = Math.min(3.0, 400.0 / e.getDistance());
        poderFinalBala = Math.max(0.1, Math.min(poder, Math.min(e.getEnergy() / 4, getEnergy() / 5)));

        //estrategia de esgotamento de energia - se mais de 10 tiros, energia maior e baixa precisao inimiga
        if (tirosInimigosDestaPartida > 10 && getEnergy() > energiaInimigo) {
            double taxaDeAcertoInimiga = (double) acertosInimigosDestaPartida / tirosInimigosDestaPartida;
            if (taxaDeAcertoInimiga < 0.15) {
                //desliga os tiros fortes, ativando o poder minimo (0.1) apenas para counterar os tiros
                //fazendo com que o inimigo gaste sua energia ate ficar desabled
                poderFinalBala = 0.1;
            }
        }

        double velocidadeBala = Rules.getBulletSpeed(poderFinalBala);

        ultimaVelocidadeInimigo = velocidadeInimigo;
        velocidadeInimigo = e.getVelocity();

        velocidadeLateralInimigo = velocidadeInimigo * Math.sin(e.getHeadingRadians() - anguloAbsoluto);
        if (velocidadeLateralInimigo != 0)
            direcaoMovimento = (velocidadeLateralInimigo > 0 ? 1 : -1);
        anguloMaximoFuga = direcaoMovimento * calcularAnguloMaximoFuga(velocidadeBala, e.getDistance());

        distanciaParede = calcularDistanciaParede(anguloAbsoluto, e.getDistance(), anguloMaximoFuga);
        distanciaParedeOposta = calcularDistanciaParede(anguloAbsoluto, e.getDistance(), -anguloMaximoFuga);

        double tempoMovimento = velocidadeBala * tempoUltimaMudancaVelocidade++ / e.getDistance();
        if (Math.abs(velocidadeInimigo) - Math.abs(ultimaVelocidadeInimigo) != 0) {
            tempoUltimaMudancaVelocidade = 0;
        }

        Robotnikka.OndaTiro ondaTiro = new Robotnikka.OndaTiro();
        ondaTiro.origemBala = minhaPosicao;
        ondaTiro.origemInimigo = projetarCoordenadas(posicaoInimigo, e.getHeadingRadians(), -velocidadeInimigo);
        ondaTiro.ultimaPosicaoInimigo = ondaTiro.origemInimigo;
        ondaTiro.anguloBala = calcularAnguloAbsoluto(ondaTiro.origemBala, ondaTiro.origemInimigo);
        ondaTiro.velocidadeBala = velocidadeBala;
        ondaTiro.tempoDisparo = getTime() - 1;
        ondaTiro.ultimoTempo = ondaTiro.tempoDisparo;
        ondaTiro.anguloMaximoFuga = anguloMaximoFuga;

        double velAproximacaoInimigo = velocidadeInimigo * Math.cos(e.getHeadingRadians() - anguloAbsoluto);

        ondaTiro.caracteristicas = new double[] {
                limitarValor(0, 1, e.getDistance() / 800.0) * 3.0,
                limitarValor(0, 1, Math.abs(velocidadeLateralInimigo) / 8.0) * 4.0,
                limitarValor(0, 1, Math.abs(velocidadeInimigo) / 8.0) * 1.0,
                limitarValor(0, 1, tempoMovimento / 1.5) * 1.0,
                limitarValor(0, 1, distanciaParede / 1.5) * 2.0,
                limitarValor(0, 1, (velAproximacaoInimigo + 8.0) / 16.0) * 1.0
        };
        ondasTiro.add(ondaTiro);

        for (int i = 0; i < ondasTiro.size(); i++) {
            Robotnikka.OndaTiro onda = ondasTiro.get(i);
            if (onda.atualizar(getTime(), posicaoInimigo)) {
                ondasTiro.remove(onda);
                i--;
            }
        }

        int melhorIndice = FATOR_TIRO_CENTRAL;
        double pesoMaximo = 0;
        double[] binsSuavizados = new double[TOTAL_FATORES_TIRO];

        // Se o KNN ja tem dados maduros, roda a busca
        if (arvoreTiro.tamanho() > 50) {
            List<Robotnikka.ArvoreKd.Entrada<Double>> vizinhos = arvoreTiro.vizinhoMaisProximo(ondaTiro.caracteristicas,
                    (int) Math.min(Math.max(5, arvoreTiro.tamanho() / 10.0), 100), false);
            double larguraBanda = (arvoreTiro.tamanho() < 100 ? 0.2 : 0.05) + (0.1 * (e.getDistance() / 800.0));

            for (int i = 0; i < TOTAL_FATORES_TIRO; i++) {
                double gfAlvo = (double) (i - FATOR_TIRO_CENTRAL) / FATOR_TIRO_CENTRAL;
                for (Robotnikka.ArvoreKd.Entrada<Double> vizinho : vizinhos) {
                    double pesoDistancia = 1.0 / (1.0 + vizinho.distancia);
                    double diffGf = gfAlvo - vizinho.valor;
                    double pesoKernel = Math.exp(-0.5 * Math.pow(diffGf / larguraBanda, 2));
                    binsSuavizados[i] += pesoDistancia * pesoKernel;
                }
            }
        }

        // Fator de Mesclagem Hibrida: Comeca 1.0 no array Frio e decai ate 0.0 no no 150
        double pesoKNN = limitarValor(0.0, 1.0, (arvoreTiro.tamanho() - 50.0) / 100.0);
        double pesoFrio = 1.0 - pesoKNN;

        for (int i = 0; i < TOTAL_FATORES_TIRO; i++) {
            // Mistura o bin do KNN com o bin Estatico do Cold Start
            double valorHibrido = (binsSuavizados[i] * pesoKNN) + (hibridoTiroFrio[i] * pesoFrio);

            if (valorHibrido > pesoMaximo) {
                pesoMaximo = valorHibrido;
                melhorIndice = i;
            }
        }

        Point2D.Double proximaMinhaPosicao = projetarCoordenadas(minhaPosicao, getHeadingRadians(), getVelocity());
        Point2D.Double proximaPosicaoInimigo = projetarCoordenadas(posicaoInimigo, e.getHeadingRadians(),
                velocidadeInimigo);
        double proximoAnguloAbsoluto = calcularAnguloAbsoluto(proximaMinhaPosicao, proximaPosicaoInimigo);

        if (getGunHeat() < getGunCoolingRate() * 3) {
            giroFinalArma = Utils.normalRelativeAngle(proximoAnguloAbsoluto - getGunHeadingRadians()
                    + anguloMaximoFuga * (melhorIndice / (double) FATOR_TIRO_CENTRAL - 1));
        } else {
            giroFinalArma = Utils.normalRelativeAngle(proximoAnguloAbsoluto - getGunHeadingRadians());
        }

        // --- LÓGICA DE BULLET SHIELDING ---
        boolean escudoAtivado = false;
        if (!ondasInimigas.isEmpty() && getGunHeat() == 0) {
            Robotnikka.OndaInimiga ondaCritica = obterOndaMaisProxima();
            if (ondaCritica != null) {
                double tempoImpacto = (minhaPosicao.distance(ondaCritica.posicaoDisparo) - ondaCritica.distanciaPercorrida) / ondaCritica.velocidadeBala;
                double perigoAtual = calcularPerigo(ondaCritica, direcoesSurf.isEmpty() ? 1 : direcoesSurf.get(0));

                // Condição de Escudo: Bala está perto (< 15 ticks), é forte (bala lenta/pesada) e o perigo é muito alto
                if (tempoImpacto > 0 && tempoImpacto < 15 && ondaCritica.velocidadeBala < 14.0 && perigoAtual > 5.0) {
                    double anguloParaBala = calcularAnguloAbsoluto(minhaPosicao, ondaCritica.posicaoDisparo);
                    giroFinalArma = Utils.normalRelativeAngle(anguloParaBala - getGunHeadingRadians());
                    poderFinalBala = 0.1; // Poder mínimo para interceptação rápida
                    escudoAtivado = true;
                }
            }
        }

        setTurnGunRightRadians(giroFinalArma);
        if (poderFinalBala > 0 && getGunHeat() == 0) {
            setFire(poderFinalBala);
        }

        setTurnRadarRightRadians(Utils.normalRelativeAngle(anguloAbsoluto - getRadarHeadingRadians()) * 2);
    }

    /**
     * Remove ondas que ja foram colididas ou destruidas no ar.
     */
    public void onBulletHitBullet(BulletHitBulletEvent e) {
        if (!ondasInimigas.isEmpty()) {
            for (int i = 0; i < ondasInimigas.size(); i++) {
                Robotnikka.OndaInimiga onda = ondasInimigas.get(i);
                if (Math.abs(onda.distanciaPercorrida - minhaPosicao.distance(onda.posicaoDisparo)) < 80) {
                    ondasInimigas.remove(i);
                    break;
                }
            }
        }
    }

    /**
     * Dispara um evento de quando o robo toma dano e alimenta o algoritmo de aprendizado.
     */
    public void onHitByBullet(HitByBulletEvent e) {
        acertosInimigosDestaPartida++; // Rastreia o acerto do inimigo para estatisticas de Cauterizacao

        if (!ondasInimigas.isEmpty()) {
            Point2D.Double posicaoAcerto = new Point2D.Double(e.getBullet().getX(), e.getBullet().getY());
            Robotnikka.OndaInimiga ondaAcertou = null;

            for (int x = 0; x < ondasInimigas.size(); x++) {
                Robotnikka.OndaInimiga onda = ondasInimigas.get(x);
                if (Math.abs(onda.distanciaPercorrida - minhaPosicao.distance(onda.posicaoDisparo)) < 50
                        && Math.abs(Rules.getBulletSpeed(e.getBullet().getPower()) - onda.velocidadeBala) < 0.001) {
                    ondaAcertou = onda;
                    break;
                }
            }

            if (ondaAcertou != null) {
                double diffAngulo = calcularAnguloAbsoluto(ondaAcertou.posicaoDisparo, posicaoAcerto)
                        - ondaAcertou.anguloDireto;
                double gf = Utils.normalRelativeAngle(diffAngulo) / calcularAnguloMaximoFuga(ondaAcertou.velocidadeBala, minhaPosicao.distance(ondaAcertou.posicaoDisparo))
                        * ondaAcertou.direcao;
                arvoreSurf.adicionarPonto(ondaAcertou.caracteristicas, limitarValor(-1.0, 1.0, gf));

                // Alimenta heuristica rapida de Surfing
                int indexGf = (int) Math.round((limitarValor(-1.0, 1.0, gf) * FATOR_MOVIMENTO_CENTRAL) + FATOR_MOVIMENTO_CENTRAL);
                indexGf = (int) Math.max(0, Math.min(TOTAL_FATORES_MOVIMENTO - 1, indexGf));
                for (int x = 0; x < TOTAL_FATORES_MOVIMENTO; x++) {
                    hibridoMovimentoFrio[x] += 1.0 / (Math.pow(indexGf - x, 2) + 1);
                }

                ondasInimigas.remove(ondaAcertou);
            }
        }
    }

    /**
     * Calcula a distancia ate atingir uma parede.
     */
    private double calcularDistanciaParede(double anguloAbsoluto, double distanciaInimigo, double envelope) {
        double dist = 1.5;
        while (dist >= 0.1) {
            dist -= 0.1;
            Point2D.Double pos = projetarCoordenadas(minhaPosicao, anguloAbsoluto + dist * envelope, distanciaInimigo);
            if (campoBatalha.contains(pos))
                break;
        }
        return dist;
    }

    /**
     * Calcula o perigo de tomar dano para cada direcao de fuga e foge para o caminho mais seguro calculado
     */
    public void executarSurfing() {
        Robotnikka.OndaInimiga ondaSurf = obterOndaMaisProxima();
        if (ondaSurf == null)
            return;

        double perigoEsquerda = calcularPerigo(ondaSurf, -1);
        double perigoDireita = calcularPerigo(ondaSurf, 1);

        double anguloAlvo = calcularAnguloAbsoluto(ondaSurf.posicaoDisparo, minhaPosicao);

        double vicioRecuo = 0;
        // Energia removida para evitar desestabilizacao do KNN (Cold Start / Energy Snowball)

        double minDistanciaParede = Math.min(Math.min(minhaPosicao.x, larguraCampo - minhaPosicao.x),
                Math.min(minhaPosicao.y, alturaCampo - minhaPosicao.y));
        if (minDistanciaParede < 120) {
            vicioRecuo = getTime() < 50 ? -0.25 : -0.1; // Breakout reativo inicial
        }

        if (perigoEsquerda < perigoDireita) {
            anguloAlvo = suavizarMovimentoParede(minhaPosicao, anguloAlvo - QUASE_METADE_PI - vicioRecuo, -1);
        } else {
            anguloAlvo = suavizarMovimentoParede(minhaPosicao, anguloAlvo + QUASE_METADE_PI + vicioRecuo, 1);
        }
        configurarFrenteTras(this, anguloAlvo);
    }

    /**
     * Remove ondas que ja passaram do robo para otimizar o processamento dos dados
     */
    public void atualizarOndas() {
        for (int i = 0; i < ondasInimigas.size(); i++) {
            Robotnikka.OndaInimiga onda = ondasInimigas.get(i);
            onda.distanciaPercorrida = (getTime() - onda.tempoDisparo) * onda.velocidadeBala;

            if (!onda.perfilCalculado && arvoreSurf.tamanho() > 0) {
                List<Robotnikka.ArvoreKd.Entrada<Double>> vizinhos = arvoreSurf.vizinhoMaisProximo(onda.caracteristicas,
                        (int) Math.min(Math.max(5, arvoreSurf.tamanho() / 10.0), 100), false);
                double larguraBanda = 0.05;
                for (int indiceGf = 0; indiceGf < TOTAL_FATORES_MOVIMENTO; indiceGf++) {
                    double gfAlvo = (double) (indiceGf - FATOR_MOVIMENTO_CENTRAL) / FATOR_MOVIMENTO_CENTRAL;
                    double perigo = 0;
                    for (Robotnikka.ArvoreKd.Entrada<Double> vizinho : vizinhos) {
                        double pesoDistancia = 1.0 / (1.0 + vizinho.distancia);
                        double diffGf = gfAlvo - vizinho.valor;
                        double pesoKernel = Math.exp(-0.5 * Math.pow(diffGf / larguraBanda, 2));
                        perigo += pesoDistancia * pesoKernel;
                    }
                    onda.perfilPerigo[indiceGf] = perigo;
                }
                onda.perfilCalculado = true;
            }

            if (onda.distanciaPercorrida > minhaPosicao.distance(onda.posicaoDisparo) + 50) {
                double diffAngulo = calcularAnguloAbsoluto(onda.posicaoDisparo, minhaPosicao) - onda.anguloDireto;
                double gf = Utils.normalRelativeAngle(diffAngulo) / calcularAnguloMaximoFuga(onda.velocidadeBala,
                        minhaPosicao.distance(onda.posicaoDisparo)) * onda.direcao;
                arvoreSurf.adicionarPonto(onda.caracteristicas, limitarValor(-1.0, 1.0, gf));

                // Alimenta heuristica rapida de Surfing
                int indexGf = (int) Math.round((limitarValor(-1.0, 1.0, gf) * FATOR_MOVIMENTO_CENTRAL) + FATOR_MOVIMENTO_CENTRAL);
                indexGf = (int) Math.max(0, Math.min(TOTAL_FATORES_MOVIMENTO - 1, indexGf));
                for (int x = 0; x < TOTAL_FATORES_MOVIMENTO; x++) {
                    hibridoMovimentoFrio[x] += 1.0 / (Math.pow(indexGf - x, 2) + 1);
                }

                ondasInimigas.remove(i);
                i--;
            }
        }
    }

    /**
     * Retorna a onda inimiga mais proxima que ainda pode ser evitada.
     */
    public OndaInimiga obterOndaMaisProxima() {
        double menorDistancia = Double.POSITIVE_INFINITY;
        Robotnikka.OndaInimiga ondaSurf = null;
        for (int i = 0; i < ondasInimigas.size(); i++) {
            Robotnikka.OndaInimiga onda = ondasInimigas.get(i);
            double distancia = minhaPosicao.distance(onda.posicaoDisparo) - onda.distanciaPercorrida;
            if (distancia > onda.velocidadeBala && distancia < menorDistancia) {
                ondaSurf = onda;
                menorDistancia = distancia;
            }
        }
        return ondaSurf;
    }

    /**
     * Simula a posicao prevista de colisao do robo com uma onda inimiga, prevendo possiveis futuros.
     */
    public FuturoPrevisto previsao(Robotnikka.OndaInimiga ondaSurf, int direcao) {
        Robotnikka.FuturoPrevisto estado = new Robotnikka.FuturoPrevisto(minhaPosicao, getHeadingRadians(), getVelocity(), getTime());
        int contador = 0;
        boolean interceptado = false;

        do {
            double anguloAlvo = calcularAnguloAbsoluto(ondaSurf.posicaoDisparo, estado.posicao);
            anguloAlvo += direcao * QUASE_METADE_PI;

            anguloAlvo = suavizarMovimentoParede(estado.posicao, anguloAlvo, direcao);

            double anguloMovimento = Utils.normalRelativeAngle(anguloAlvo - estado.direcaoAtual);
            int dirMovel = 1;

            if (Math.cos(anguloMovimento) < 0) {
                anguloMovimento += Math.PI;
                dirMovel = -1;
            }

            anguloMovimento = Utils.normalRelativeAngle(anguloMovimento);
            estado = estado.obterProximoEstado(dirMovel, anguloMovimento);
            estado = verificarColisaoParede(estado);

            // Penalidade Inercial (Delay de reaceleracao)
            if (estado.velocidade == 0.0) contador += 1;

            contador++;

            if (estado.posicao.distance(ondaSurf.posicaoDisparo) < ondaSurf.distanciaPercorrida
                    + (contador * ondaSurf.velocidadeBala) + ondaSurf.velocidadeBala) {
                interceptado = true;
            }
        } while (!interceptado && contador < 500);

        return estado;
    }

    /**
     * Calcula o perigo de tomar dano ao simular o movimento de interseccao do robo com uma onda inimiga
     */
    public double calcularPerigo(Robotnikka.OndaInimiga ondaSurf, int direcao) {
        Robotnikka.FuturoPrevisto posicaoPrevista = previsao(ondaSurf, direcao);
        double diffAngulo = calcularAnguloAbsoluto(ondaSurf.posicaoDisparo, posicaoPrevista.posicao)
                - ondaSurf.anguloDireto;
        double gf = Utils.normalRelativeAngle(diffAngulo) / calcularAnguloMaximoFuga(ondaSurf.velocidadeBala,
                minhaPosicao.distance(ondaSurf.posicaoDisparo)) * ondaSurf.direcao;
        gf = limitarValor(-1.0, 1.0, gf); //guessfactors

        int indice = (int) Math.round((gf * FATOR_MOVIMENTO_CENTRAL) + FATOR_MOVIMENTO_CENTRAL);
        indice = (int) limitarValor(0, TOTAL_FATORES_MOVIMENTO - 1, indice);

        double perigoKNN = 0;
        if (ondaSurf.perfilCalculado) {
            perigoKNN = ondaSurf.perfilPerigo[Math.max(0, indice - 1)] +
                    ondaSurf.perfilPerigo[indice] +
                    ondaSurf.perfilPerigo[Math.min(TOTAL_FATORES_MOVIMENTO - 1, indice + 1)];
        }

        double perigoFrio = hibridoMovimentoFrio[Math.max(0, indice - 1)] +
                hibridoMovimentoFrio[indice] +
                hibridoMovimentoFrio[Math.min(TOTAL_FATORES_MOVIMENTO - 1, indice + 1)];

        //implementacao hibrida do surf: transicao suave do array frio (0-50 nos) para KNN (100 nos)
        double pesoKNN = limitarValor(0.0, 1.0, (arvoreSurf.tamanho() - 50.0) / 50.0);

        //se a arvore estiver vazia, foge do centro absoluto (heuristica base de 0 nos)
        if (arvoreSurf.tamanho() == 0 && perigoFrio == 0) return Math.abs(gf);

        return (perigoKNN * pesoKNN) + (perigoFrio * (1.0 - pesoKNN));
    }

    /**
     * Reposiciona o robo caso a simulacao detecte um futuro de colisao entre ele e a parede.
     */
    public FuturoPrevisto verificarColisaoParede(Robotnikka.FuturoPrevisto estado) {
        if (!campoBatalha.contains(estado.posicao)) {
            return new Robotnikka.FuturoPrevisto(
                    new Point2D.Double(limitarValor(18.0, larguraCampo - 18.0, estado.posicao.x),
                            limitarValor(18.0, alturaCampo - 18.0, estado.posicao.y)),
                    estado.direcaoAtual, 0.0, estado.tempoJogo); // Colidir zera a velocidade na fisica do jogo
        }
        return estado;
    }

    /**
     * Ajusta o angulo do movimento do robo para que ele surfe entre as margens da parede
     */
    public double suavizarMovimentoParede(Point2D.Double loc, double dir, int dirOrbita) {
        dir = Utils.normalAbsoluteAngle(dir);
        double vel = 8.0;
        double x = loc.x, y = loc.y;
        double norte = alturaCampo - MARGEM_PAREDE;
        double sul = MARGEM_PAREDE;
        double leste = larguraCampo - MARGEM_PAREDE;
        double oeste = MARGEM_PAREDE;

        if (dir > Math.PI) {
            if (deveSuavizar(dir - Math.PI, vel, x - oeste, dirOrbita))
                dir = suavizarAngulo(vel, x - oeste, dirOrbita) + Math.PI;
        } else if (dir < Math.PI) {
            if (deveSuavizar(dir, vel, leste - x, dirOrbita))
                dir = suavizarAngulo(vel, leste - x, dirOrbita);
        }

        if (dir < Math.PI / 2 || dir > 3 * Math.PI / 2) {
            if (deveSuavizar(dir + Math.PI / 2, vel, norte - y, dirOrbita))
                dir = suavizarAngulo(vel, norte - y, dirOrbita) - Math.PI / 2;
        } else if (dir > Math.PI / 2 && dir < 3 * Math.PI / 2) {
            if (deveSuavizar(dir - Math.PI / 2, vel, y - sul, dirOrbita))
                dir = suavizarAngulo(vel, y - sul, dirOrbita) + Math.PI / 2;
        }

        return dir;
    }

    /**
     * Calcula o arco exato de deslocamento para "suavizar" o angulo entre o robo e a parede
     */
    private double suavizarAngulo(double vel, double distParede, int dirOrbita) {
        if (distParede < 0.01)
            return dirOrbita == 1 ? Math.PI : 0;
        double dir = Math.acos((distParede - RAIO_CURVA) / Math.sqrt(RAIO_CURVA * RAIO_CURVA + vel * vel))
                + Math.atan(vel / RAIO_CURVA);
        return (dirOrbita == 1 ? dir : Math.PI - dir);
    }

    /**
     * Testa antes se a atual trajetoria do robo colidira com a margem, exigindo uma retificacao.
     */
    private boolean deveSuavizar(double dir, double vel, double distParede, int dirOrbita) {
        distParede -= (vel * Math.sin(dir));
        if (distParede < 0)
            return true;
        distParede -= (1 + Math.sin(dir + (dirOrbita * Math.PI / 2))) * RAIO_CURVA;
        return distParede < 0;
    }

    /**
     * Modela o estado do robo em um instante Futuro para fazer a previsao de comportamentos.
     */
    static class FuturoPrevisto {
        Point2D.Double posicao;
        double direcaoAtual;
        double velocidade;
        long tempoJogo;

        FuturoPrevisto(Point2D.Double loc, double d, double v, long t) {
            this.posicao = loc;
            this.direcaoAtual = d;
            this.velocidade = v;
            this.tempoJogo = t;
        }

        public Robotnikka.FuturoPrevisto obterProximoEstado(int direcao, double curva) {
            double proximaVel = Robotnikka.Fisica.proximaVelocidade(velocidade, direcao);
            double proximaDir = direcaoAtual + Robotnikka.Fisica.incrementoCurva(curva, velocidade);
            return new Robotnikka.FuturoPrevisto(projetarCoordenadas(posicao, proximaDir, proximaVel),
                    proximaDir, proximaVel, tempoJogo + 1);
        }
    }

    /**
     * Funcoes auxiliares para calcular o comportamento fisico do ambiente do robocode.
     */
    public static class Fisica {
        public static double proximaVelocidade(double v, int d) {
            if (d == 0)
                return v - Math.signum(v) * Math.min(desaceleracao(Math.abs(v)), Math.abs(v));
            return Math.max(-8, Math.min(8, v + d * (Math.signum(v) * d < 0 ? desaceleracao(Math.abs(v)) : 1)));
        }

        public static double desaceleracao(double vel) {
            return Math.max(1, Math.min(2, 1 + vel / 2));
        }

        public static double curvaMaxima(double v) {
            return Math.PI / 18 - Math.abs(v) * Math.PI / 240;
        }

        public static double incrementoCurva(double t, double v) {
            double max = curvaMaxima(v);
            return Math.max(-max, Math.min(max, t));
        }
    }

    /**
     * Aplica o movimento de "re" quando o giro necessario for superior a 90 graus.
     */
    public static void configurarFrenteTras(AdvancedRobot robo, double anguloAlvo) {
        double angulo = Utils.normalRelativeAngle(anguloAlvo - robo.getHeadingRadians());
        if (Math.abs(angulo) > (Math.PI / 2)) {
            if (angulo < 0)
                robo.setTurnRightRadians(Math.PI + angulo);
            else
                robo.setTurnLeftRadians(Math.PI - angulo);
            robo.setBack(100);
        } else {
            if (angulo < 0)
                robo.setTurnLeftRadians(-1 * angulo);
            else
                robo.setTurnRightRadians(angulo);
            robo.setAhead(100);
        }
    }

    /**
     * Projeta uma nova coordenada baseada no ponto de origem, angulo e hipotenusa.
     */
    public static Point2D.Double projetarCoordenadas(Point2D.Double origem, double angulo, double distancia) {
        return new Point2D.Double(origem.x + Math.sin(angulo) * distancia, origem.y + Math.cos(angulo) * distancia);
    }

    /**
     * Mede o angulo absoluto do alvo relativo a um ponto zero.
     */
    public static double calcularAnguloAbsoluto(Point2D.Double origem, Point2D.Double alvo) {
        return Math.atan2(alvo.x - origem.x, alvo.y - origem.y);
    }

    /**
     * Identifica o limite maximo onde a bala poderia alcancar o inimigo com base na sua velocidade
     * estimada de 8 px/tick, incluindo a largura fisica do hitbox.
     */
    public static double calcularAnguloMaximoFuga(double velocidadeBala, double distancia) {
        return Math.asin(8.0 / velocidadeBala) + Math.atan(18.0 / Math.max(0.1, distancia));
    }

    /**
     * Limita o valor numerico entre o piso (min) e teto (max).
     */
    public double limitarValor(double min, double max, double valor) {
        return Math.max(min, Math.min(max, valor));
    }

}
