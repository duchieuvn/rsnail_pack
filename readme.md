## Usage 
0. Prerequisite  
- Docker installed  
- x11 app for linux install (optional)  
.  

1. Unzip all zip files  
.

2. Open this folder `rsnail_pack` in terminal  
.  

3. Run the following commands in terminal  
  
- Build a Docker image based on `Dockerfile`  

``` 
docker build -t rsnail_pack-all . 
```
  
- Run the container  

```
docker run -it \
  --rm \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  rsnail_pack-all \
  bash
```

4. Run `colcon build` inside the container  
- 14/15 packages should be completed

## Contact 
Hieu  
t.tran@oth-aw.de