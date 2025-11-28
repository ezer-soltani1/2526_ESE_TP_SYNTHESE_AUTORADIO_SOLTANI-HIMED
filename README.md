# TP_Synthèse - Autoradio 

HIMED Zineddine - SOLTANI Ezer

## Introduction

Ce projet s’inscrit dans le cadre du TP de synthèse portant sur la réalisation d’un système embarqué faisant office d’autoradio sur carte **STM32 NUCLEO-L476RG**. L’objectif est de créer une chaîne audio complète intégrant l’acquisition, le traitement et la restitution d’un signal sonore, avec une interface utilisateur via un shell ainsi qu’un affichage lumineux type VU-mètre.

Le système repose notamment sur FreeRTOS, un shell série via USART2, un GPIO Expander contrôlant un ensemble de LED via SPI, et le codec audio **SGTL5000**, configuré en I2C et utilisant le protocole I2S via SAI2 pour le flux audio. Le TP est structuré en plusieurs étapes successives, chacune validant une partie matérielle ou logicielle avant l'intégration finale.


## Objectifs

- Initialiser un projet embarqué sur STM32 avec FreeRTOS.
- Mettre en place un shell fonctionnant en tâche FreeRTOS, avec interruptions et driver structuré.
- Piloter un GPIO Expander via SPI pour contrôler un ensemble de LED servant de VU-mètre.
- Configurer le codec audio **SGTL5000** :
  - Configuration en I2C
  - Transfert audio en I2S via SAI2, horloge MCLK activée, DMA en mode circulaire
- Générer des signaux audio (ex : triangle) et les analyser à l'oscilloscope.
- Réaliser un bypass numérique ADC → DAC.
- Implémenter un filtre RC numérique sous forme d'équation par récurrence.
- Ajouter un effet audio numérique (distorsion, tremolo, delay, etc.).


## 1) Démarrage du projet

La première étape du TP consistait à créer un projet STM32 sous **STM32CubeIDE** pour la carte NUCLEO-L476RG sans activer la BSP afin de garder le contrôle sur la configuration matérielle. Après la génération du code, plusieurs vérifications ont été réalisées pour s'assurer du bon fonctionnement des périphériques de base avant d'aborder la partie audio plus complexe.

Nous avons d’abord testé la LED LD2 afin de valider l’accès au GPIO. L’allumage et le clignotement ont fonctionné comme prévu, confirmant la bonne configuration du microcontrôleur et du clocking de base : 

![Clignotement de la LED](images/Clignotement_LED.jpeg)


Nous avons ensuite configuré l’USART2, relié au ST-Link interne, afin de communiquer avec un terminal série sur PC. La transmission a été validée en envoyant des messages simples, puis nous avons redirigé la fonction `printf()` vers cette liaison afin de simplifier le débogage et l'affichage des logs durant la suite du projet. Ces premières étapes ont permis d’obtenir une interface de sortie fiable pour vérifier le fonctionnement des modules développés.

![Test de Printf](images/test_uart2.png)

Après validation des périphériques basiques, nous avons activé **FreeRTOS en mode CMSIS-V1** afin de travailler en environnement multitâche. Cela a permis d’isoler chaque fonctionnalité (shell, audio, effets, affichage LED) dans des tâches indépendantes tout en conservant une meilleure lisibilité et modularité du code.

Une étape essentielle consistait à mettre en place un **shell accessible via UART**.

![Test Shell](images/test_shell_tache.png)

Celui-ci s’exécute dans une tâche FreeRTOS dédiée et utilise des interruptions pour la réception série. Le shell permet d’interagir dynamiquement avec le système, notamment pour tester les différents modules (GPIO Expander, codec SGTL5000, filtres, effets, etc.). Cette approche offre une meilleure flexibilité qu’un programme à comportement figé, car elle permet de modifier les paramètres en temps réel sans recompiler.

À ce stade, le système était fonctionnel avec :
- une communication série fiable,
- un shell capable de recevoir et interpréter des commandes,
- un environnement multitâche stable,
- une interface de trace via `printf()` pour le débogage.

Cette base logicielle a servi de fondation pour l’intégration des éléments audio et du pilotage des LEDs.

## 2) Le GPIO Expander et le VU‑Mètre

### 2.1 Configuration
Le GPIO Expander utilisé dans ce projet est le **MCP23S17**, un expander SPI permettant d’ajouter 16 lignes d’E/S au microcontrôleur. Sa datasheet a été consultée afin d’identifier son mode de fonctionnement, son protocole SPI ainsi que les registres nécessaires à la configuration.

L’expander est relié au microcontrôleur via le **périphérique SPI3** du STM32. Les broches utilisées sont :
- **PC10** → SCK (horloge SPI)
- **PC11** → MISO (lecture depuis le MCP23S17)
- **PC12** → MOSI (écriture vers le MCP23S17)
- Une broche GPIO libre → CS (Chip Select), configurée en sortie

La configuration du SPI3 a été réalisée directement dans STM32CubeIDE. Les paramètres visibles dans la capture ci-dessous montrent la configuration exacte utilisée pour communiquer avec le MCP23S17.

![Configuration SPI3](images/spi3_config.png)

Cette configuration assure un dialogue fiable entre le STM32 et le MCP23S17. Une fois les paramètres confirmés, le code a été généré automatiquement par STM32CubeIDE.

### 2.2 Tests

La validation du fonctionnement du GPIO Expander MCP23S17 et des LED a été réalisée à l’aide de la tâche `LedTask()`. Cette tâche est chargée d’initialiser l’expander, puis de générer un chenillard sur l’ensemble des sorties, ce qui permet à la fois de faire clignoter une LED et de tester successivement toutes les lignes des ports A et B.

Le déroulement de la tâche est le suivant :

* Un **reset matériel** est d’abord appliqué au MCP23S17 via la broche `VU_nRESET` (mise à 0 puis à 1 avec un léger délai), afin de garantir un état propre du composant.
* Les deux ports du MCP23S17 sont ensuite **configurés en sortie** en écrivant `0x00` dans les registres `IODIRA` et `IODIRB` via l’interface SPI3.
* Une variable `GPIO_value` est initialisée à `0x01`. Cette valeur représente le motif binaire envoyé vers les LED.
* Dans une boucle infinie, cette valeur est écrite dans les registres de latch de sortie `OLATA` et `OLATB`, de sorte que le même motif soit appliqué sur les ports A et B.
* Un délai de 100 ms est appliqué entre chaque mise à jour, pour rendre le mouvement des LED visible.
* À chaque itération, `GPIO_value` est décalée d’un bit vers la gauche (`GPIO_value <<= 1`). Lorsque la valeur devient nulle (débordement après le bit le plus significatif), elle est réinitialisée à `0x01`.

Ce fonctionnement produit un **chenillard** : un seul bit à `1` se déplace de la position la plus faible à la plus forte, puis revient au début. Visuellement, cela se traduit par une LED allumée qui se déplace le long de la barre de LED, ce qui permet de vérifier individuellement chaque sortie de l’expander sur les ports A et B.

La tâche utilisée est la suivante :

```c
void LedTask(void *argument)
{
    // Reset
    HAL_GPIO_WritePin(VU_nRESET_GPIO_Port, VU_nRESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(VU_nRESET_GPIO_Port, VU_nRESET_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    uint8_t tx_data[3];

    // Configurer Port A en sortie
    tx_data[0] = MCP_OPCODE_WRITE;
    tx_data[1] = MCP_IODIRA;
    tx_data[2] = 0x00; // Tous les pins en sortie
    HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, tx_data, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    // Configurer Port B en sortie
    tx_data[1] = MCP_IODIRB;
    tx_data[2] = 0x00;
    HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, tx_data, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);

    tx_data[0] = MCP_OPCODE_WRITE;

    uint8_t GPIO_value = 0x01;
    for(;;)
    {
        tx_data[2] = GPIO_value;

        // Port A
        tx_data[1] = MCP_OLATA;
        HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi3, tx_data, 3, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);

        // Port B
        tx_data[1] = MCP_OLATB;
        HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi3, tx_data, 3, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);

        HAL_Delay(100);

        GPIO_value <<= 1;
        if (GPIO_value == 0x00 ) {
            GPIO_value = 0x01;
        }
    }
}
```

```
}
```

}
