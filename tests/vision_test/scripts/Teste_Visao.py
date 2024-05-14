import cv2
import numpy as np

image = cv2.imread('C:/Users/Vicente Pereira/Desktop/caixa pequena.png')
cv2.imshow('image', image)

#um valor de 30 permite distinguir a cor preta da caixa
ret,th1 = cv2.threshold(image,30,255,cv2.THRESH_BINARY)
cv2.imshow("binary", th1)

#ao aplicar o gradiente obtemos o contorno da caixa
ee = np.ones((3,3),np.uint8)
gra = cv2.morphologyEx(th1, cv2.MORPH_GRADIENT, ee)
cv2.imshow("Gradient", gra)


height, width, _ = image.shape #obtem a altura e largura da imagem


for y in range(height):
    for x in range(width):

        pixel_value = gra[y, x] # obtem o valor do pixel na posição x,y

        # verifica os canais rgb se têm valores menores ou iguais a 10
        if all(channel >= 10 for channel in pixel_value):
            print("Pixel na posição (", x, ",", y, "):", pixel_value)

# conta o nmr de pixeis brancos na imagem
white_pixels = np.sum(gra == 255)
print("Quantidade de pixeis brancos:", white_pixels)

if(white_pixels > 9000):
    print("Caixa grande!")
elif(white_pixels < 9000):
        print("Caixa pequena!")

cv2.waitKey(0)
cv2.destroyAllWindows()