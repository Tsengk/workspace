const http = require('http');

let fruits = [
    {id: 1,name: "苹果",price: 5},
    {id: 2,name: "香蕉",price: 3},
    {id: 3,name: "橘子",price: 4}
];

const server = http.createServer(function(request,response) {
    response.setHeader('Access-Control-Allow-Origin', '*');
    response.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTIONS');
    response.setHeader('Access-Control-Allow-Headers','Conten-Type');
    response.setHeader('Content-Type','application/json; charset=utf-8');

    if(request.method === 'OPTIONS') {
        response.writeHead(200);
        response.end();
        return;
    }

    if (request.url ==='/api/fruits' && request.method === 'GET') {
        response.end(JSON.stringify(fruits));
    }else if (request.url === '/api/fruits' && request.method === 'POST') {
        let body = ' ';

        request.on('data', chunk => {
            body += chunk.toString();
        });

        request.on('end', () => {
            const newFruit = JSON.parse(body);

            newFruit.id = fruits.length + 1;
            fruits.push(newFruit);

            console.log("收到新水果：" , newFruit);

            response.end(JSON.stringify({message: "添加成功!"}));
        })
    }else {
        response.writeHead(404);
        response.end(JSON.stringify({message: "没有该接口"}));
    }
});

server.listen(3000, '0.0.0.0',function() {
    console.log("服务器已经启动");
    console.log("监听端口：http://localhost:3000");
});