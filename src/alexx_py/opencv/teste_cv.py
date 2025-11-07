import cv2

# Caminho para a sua imagem (substitua pelo caminho real)
image_path = '/home/kifelix/Downloads/teste_pista1.png'  # Exemplo: '/home/user/floor_image.jpg'
# 1. Ler a imagem

image = cv2.imread(image_path)
# Verificar se a imagem foi carregada

if image is None:
    print("Erro: Não foi possível carregar a imagem. Verifique o caminho.")
    exit()

# 2. Converter para escala de cinza
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 3. Aplicar thresholding para identificar a linha branca
# Threshold: pixels acima de 200 (aprox. branco) ficam brancos (255), outros pretos (0)
# Ajuste o valor 200 conforme a iluminação da sua imagem (teste valores entre 150-250)
thresh_value = 200
_, binary_image = cv2.threshold(gray_image, thresh_value, 255, cv2.THRESH_BINARY)

# (Opcional) Aplicar um filtro para suavizar ruídos (ex.: Gaussian Blur antes do threshold)
# gray_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
# _, binary_image = cv2.threshold(gray_image, thresh_value, 255, cv2.THRESH_BINARY)
# 4. Mostrar as imagens (para visualização)
cv2.imshow('Imagem Original', image)
cv2.imshow('Imagem em Cinza', gray_image)
cv2.imshow('Linha Detectada (Binarizada)', binary_image)

# Aguardar tecla para fechar (pressione qualquer tecla)
cv2.waitKey(0)
cv2.destroyAllWindows()

# (Opcional) Salvar a imagem binarizada
cv2.imwrite('linha_detectada.jpg', binary_image)
print("Imagem binarizada salva como 'linha_detectada.jpg'")	
