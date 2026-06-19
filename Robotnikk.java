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

}
