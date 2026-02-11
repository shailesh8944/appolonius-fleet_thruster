FROM abhilashiit/oe5005:1.0

# Install CasADi and tqdm into the image
RUN pip install --no-cache-dir --upgrade pip
RUN pip install --no-cache-dir casadi tqdm
