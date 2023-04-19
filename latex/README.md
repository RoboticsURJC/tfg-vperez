# Plantilla LaTeX para la escritura del TFG o TFM
## (CC-BY-NC-SA) Julio Vega

El objetivo de esta **plantilla de LaTeX** es facilitar a los estudiantes la escritura de su trabajo de fin de grado (TFG) o de máster (TFM) en este sistema profesional de confección de documentos.

Esta plantilla puede ser usada tanto en *Windows* como en *Linux*. El diseño incluye el logo de la Universidad Rey Juan Carlos (URJC), pero también es extendible a otras universidades, simplemente cambiando este. Asimismo, la estructura de capítulos está enfocada a carreras técnicas, pero puede servir de mucha ayuda para el trabajo de cualquier otra rama.

## 0. Introducción a LaTeX

Si bien *Microsoft Word* es la suite ofimática por excelencia, no tiene la potencia suficiente para la confección de un documento largo o científico. *LaTeX* es mucho más potente, aunque tiene una pequeña curva logarítmica inicial de aprendizaje; esto es, difícil al comienzo hasta que se le coge soltura.

De forma muy resumida, la principal diferencia entre *Microsoft Word* (o *LibreOffice Writer*) y *LaTeX* es que el primero es un procesador de textos y, el segundo, un sistema de composición de textos. Así, en *Word*, a medida que escribimos, vamos viendo el resultado final en el propio documento. Por ejemplo, podemos modificar los estilos y tamaño de la fuente y, automáticamente, vemos cómo afectan esos cambios; de igual forma, podemos insertar una imagen, la movemos y posicionamos directamente donde deseemos.

En *LaTeX*, el proceso de confección del documento es bien distinto. Lo que escribimos no es el documento en sí, sino instrucciones de cómo queremos que sea el documento. Es por ello que tenemos que conocer esas instrucciones y la sintaxis de estas. Para ver el documento renderizado (en bonito), pasamos este documento "crudo" por el motor de *LaTeX*, y es en este paso donde se produce la magia. A este proceso se le denomina compilar (que se describe en la siguiente sección).

Inicialmente, hasta que cogemos soltura con el lenguaje de *LaTeX*, la escritura puede resultar tediosa. Entonces, ¿por qué usar este sistema? Fácil: porque el resultado final es sencillamente brillante, perfecto. ¿La razón de esta perfección? Simplemente porque la maquetación no depende de nuestra habilidad, sino que la confecciona una máquina.

## 1. Instalación y uso

### 1.1. Windows

Lo más cómodo para poder trabajar con documentos *LaTeX* en *Windows* es descargarse la distribución *MikTeX*, que incluye todo: el editor y el motor de este sistema (compilador y paquetes). Para su instalación y uso, sigue los pasos siguientes:

1. Descarga *MiKTeX* de aquí: https://miktex.org/download
2. Lanza el ejecutable descargado. Si te aparece la ventana de seguridad de Windows que titula *"Windows protegió su equipo"*, haz clic en *"Más info"* y entonces clic en el botón *"Ejecutar de todos modos"*. Esto ocurre porque el editor de *MiKTeX* no está en la base de datos de los reconocidos por Windows.
3. Ve haciendo clic sucesivamente en *"Siguiente"* hasta acabar el proceso de instalación.
4. Una vez instalado, tendremos lanzada automáticamente la consola de *MiKTex*. Nos aparecerá en la barra de tareas. Pulsamos en tal icono con el botón derecho del ratón y seleccionamos la opción de *"Check for updates"*, para tener siempre la última versión.
5. Abrimos el editor que también se ha instalado con *MiKTeX* y que se llama *"TeXworks"*. Con este, abrimos *"memoria.tex"*. A la izquierda veremos el contenido en crudo y, a la derecha, el resultado final.
6. Para obtener ese resultado final, pulsamos el icono de *"Play"* que aparece en la parte superior, asegurándonos previamente de que la opción marcada en el desplegable de su derecha sea *"pdfLaTeX+MakeIndex+BibTeX"*.
La primera vez que compilemos, como faltarán todas las librerías usadas en el documento, el compilador nos avisará de ello mediante una ventana de *"Package Installation"*.
Esta ventana aparecerá por cada librería que nos falte, que serán bastantes. Por ello, si queremos que no nos aparezca tal ventana reiteradamente y que lo haga de forma automática, es muy recomendable desmarcar la opción *"Always show the dialog before installing packages"* y entonces sí, hacemos clic en *"Install"*.
Podemos seguir todo el proceso de instalación de librerías en la *"Consola de salida"* que aparece en la parte inferior izquierda del editor. Veremos aparecer mensajes como: *"downloading http://..."* o *"extracting files ..."* o *"C:\Users\...\MiKTeX\...\xxx.sty"*.
7. Una vez finalizado el proceso, ya tendremos el resultado final en *.pdf*, que nos aparecerá en la parte derecha de la pantalla. Con este podremos, entre otras cosas:
  - Imprimirlo en *.pdf*: vamos a *"Archivo -> Imprimir PDF"*, y ya tendremos nuestro documento *"presentable"*.
  - Manejarlo con el visualizador que usemos habitualmente: en *"Scripts -> Open in default viewer"*. Previamente hemos de darle permisos para ello: *"Editar -> Preferencias -> Scripts -> Autorizar que los scripts puedan ejecutar comandos del sistema"*.

### 1.2. Linux

Para poder usar esta plantilla en *Linux* solo necesitas (si es que no las tienes ya) las librerías de Latex. Para instalarlas, ejecuta el siguiente comando en tu terminal:

```bash
sudo apt install bibtex2html texlive-latex-base texlive-latex-extra texlive-generic-extra \
texlive-font-utils texlive-fonts-recommended texlive-fonts-extra texlive-lang-spanish \
texlive-science texlive-bibtex-extra aspell-es texlive-extra-utils biber
```

Respecto a cómo editar un fichero *LaTeX* en *Linux*, puedes usar cualquier editor estándar, como *Gedit*, o alguno más complejo y especializado en *LaTeX*, como *Kile*.

Una vez editado, para ver el resultado final en *.pdf*, es necesaria su traducción (compilación). Para facilitar esta compilación, simplemente ejecuta *make* desde el *Terminal*; pues se proporciona un fichero *Makefile*, que incluye esa y otras funcionalidades descritas a continuación:

a) Compilar:

```bash
make
```

b) Limpiar los archivos auxiliares generados en la compilación:

```bash
make clean
```

c) Hacer una copia de seguridad:

```bash
make backup
```
Se generará un fichero como este: *"2020.10.27.tgz"*, que incluye todos los ficheros *.tex* y las figuras.

d) Guardar una copia completa:

```bash
make release
```
Se generará un fichero como este: *"2020.10.27.tgz"*, que incluye todos los ficheros y carpetas.

## 2. Estructura de ficheros

### 2.1. Raíz

#### 2.1.1. Carpetas

**capitulos**: aquí están los ficheros Latex correspondientes a los capítulos.

**portada**: aquí están los ficheros Latex correspondientes a las primeras páginas del documento.

**figs**: aquí deberás meter todas las figuras que enlaces en el documento.

#### 2.1.2. Ficheros que tendrás que editar

**memoria.tex**: fichero principal. Desde él se acceden al estilo, páginas de portada, capítulos y bibliografía (estilo establecido: APA).

**estilo.tex**: define el estilo/apariencia que tendrá el documento final.

**bibliografia.bib**: incluye las referencias bibliográficas.

#### 2.1.3. Ficheros auxiliares que no tendrás que editar

**Makefile**: archivo de compilación.

**listings.sty**, **lstmisc.sty**: paquetes de definición y estilo de "listings".

### 2.2. capitulos ###

Los ficheros Latex aquí contenidos siguen la siguiente estructura:

**capitulo1.tex**: introducción.

**capitulo2.tex**: objetivos.

**capitulo3.tex**: plataforma de desarrollo.

**capitulo4.tex**: diseño.

**capitulo5.tex**: conclusiones.

### 2.3. portada ###

En esta carpeta se encuentran los ficheros Latex correspondientes a las primeras páginas del documento.

**indice.tex**: fichero principal. Desde él se acceden al resto de ficheros de esta carpeta en el siguiente orden. Además, recoge también la tabla de contenidos, lista de figuras, lista de códigos, lista de ecuaciones y lista de tablas (o cuadros).

**portada.tex**: página de portada del documento.

**licencia.tex**: página dedicada a especificar la licencia del documento. Por defecto está bajo licencia CC-BY-NC-SA.

**agradecimientos.tex**: página de agradecimientos.

**resumen.tex**: resumen del trabajo.

**acronimos.tex**: donde se especifican los acrónimos usados en el documento.

