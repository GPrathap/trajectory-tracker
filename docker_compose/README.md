# Trajectory Tracker Docker Container 

## Prerequisites

* docker, docker compose, nvidia-docker container toolbox 
* (Optional) WSLg
* If you have GPU uncomment the following in the docker-compose.yml
    ```deploy:
      resources:
        reservations:
           devices:
             - capabilities: [gpu]
             -  count: all
## Getting started

```bash
git clone https://github.com/GPrathap/trajectory-tracker.git
cd trajectory-tracker/docker_compose
docker-compose build --no-cache --progress=plain
docker-compose up -d  --remove-orphans
docker-compose exec trajectory_tracker bash
```

### docker-compose

```bash
docker-compose stop
```

```bash
docker-compose down
```

```bash
docker-compose exec trajectory_tracker bash
```

### Inside the docker container 
#### set up github ssh key 
https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent 


