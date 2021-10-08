1、create a new repository on the command line
echo "# robot416" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/guozi-1997/robot416.git
git push -u origin main

2、 push an existing repository from the command line
git remote add origin https://github.com/guozi-1997/robot416.git
git branch -M main
git push -u origin main