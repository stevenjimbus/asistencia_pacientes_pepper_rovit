 #!/bin/bash         
ssh-add ~/.ssh/id_rsa
echo "Push a github"
git add -A
git commit -m "Fecha: $(date +%d-%b-%H_%M)"
git push -u origin master
 

echo "Push finalizado"

