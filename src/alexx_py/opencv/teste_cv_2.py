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

# 3. Aplicar GaussianBlur para reduzir ruídos (melhoria opcional)
# Kernel de 5x5 (ajustável: valores maiores suavizam mais, mas podem borrar detalhes)
blur_kernel = (5, 5)
blurred_image = cv2.GaussianBlur(gray_image, blur_kernel, 0)

# 4. Aplicar thresholding para identificar a linha branca
# Threshold: pixels acima de 200 (aprox. branco) ficam brancos (255), outros pretos (0)
# Ajuste o valor 200 conforme a iluminação da sua imagem (teste valores entre 150-250)
thresh_value = 200
_, binary_image = cv2.threshold(blurred_image, thresh_value, 255, cv2.THRESH_BINARY)

# 5. Detecção avançada: Encontrar contornos da linha binarizada (melhoria opcional)
# cv2.findContours retorna uma lista de contornos (formas detectadas)
contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Criar uma cópia da imagem original para desenhar os contornos
image_with_contours = image.copy()
# Desenhar os contornos em vermelho (cor: (0, 0, 255), espessura: 2)
cv2.drawContours(image_with_contours, contours, -1, (0, 0, 255), 2)

# (Opcional) Alternativa ao threshold: Usar Canny Edge Detection para bordas
# Melhor para iluminação variável ou sombras (detecta gradientes)
# Descomente as linhas abaixo para testar em vez do threshold
# canny_low = 50  # Limite inferior para bordas (ajustável)
# canny_high = 150  # Limite superior (ajustável)
# edges = cv2.Canny(blurred_image, canny_low, canny_high)
# cv2.imshow('Bordas Detectadas (Canny)', edges)

# 6. Mostrar as imagens (para visualização)
cv2.imshow('Imagem Original', image)
cv2.imshow('Imagem em Cinza', gray_image)
cv2.imshow('Imagem Suavizada (Blur)', blurred_image)
cv2.imshow('Linha Detectada (Binarizada)', binary_image)
cv2.imshow('Contornos Desenhados na Original', image_with_contours)

# Aguardar tecla para fechar (pressione qualquer tecla)
cv2.waitKey(0)
cv2.destroyAllWindows()

# (Opcional) Salvar as imagens processadas
cv2.imwrite('linha_detectada.jpg', binary_image)
cv2.imwrite('contornos_desenhados.jpg', image_with_contours)
print("Imagens salvas: 'linha_detectada.jpg' e 'contornos_desenhados.jpg'")

