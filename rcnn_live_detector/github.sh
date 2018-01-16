 #!/bin/bash         
ssh-add ~/.ssh/id_rsa
echo "Push a Bitbucket..."
git add -A
git commit -m "Fecha: $(date +%d-%b-%H_%M)"
git push origin master
 

echo "Push finalizado"

