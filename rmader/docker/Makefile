all: build run

build:
	cd /home/kkondo/rmader_project/rmader_ws/src
	docker build -f rmader/rmader/docker/Dockerfile -t rmader .
run:
	docker run --volume=/home/kkondo/rmader_project/rmader_ws/src/rmader/rmader/docker/gurobi.lic:/opt/gurobi/gurobi.lic:ro --volume=/home/kkondo/data:/home/kota/data -it rmader
	docker exec -w /home/rmader_ws/src/rmader/rmader/scripts rmader python run_many_sims_multi_agent.py