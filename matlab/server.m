function s = server(host, port,Mdl,avgDB)
    s.Q = Queue();
    s.socket = tcpip(host,port,'NetworkRole','Server');
    set(s.socket, 'InputBufferSize', 3000000);
    set(s.socket, 'OutputBufferSize', 3000000);
    fopen(s.socket);

    s.send = @send;
    function send(data)
        d = whos(data);
        fwrite(s.socket, data, d.class);
    end

    s.receive = @receive;
    function receive()
        TimerFcn = {@recv, s.socket, s.Q};
        t = timer('ExecutionMode', 'FixedRate', ...
            'Period', 1, ...
            'TimerFcn', TimerFcn);
        start(t);
    end

    s.disconnect = @disconnect;
    function disconnect()
        fclose(s.socket);
        delete(s.socket);
        clear s.socket;
    end

    s.findLocation = @findLocation;
    function [our_position,P1] = findLocation(in) 
        % get the sample
        sample = str2num(in); 
        
        % predict with MATLAB Knn
        our_position = predict(Mdl,sample);  
        
        % predict from average DB table
        [min_val,P1] = min(sum((table2array(avgDB(:,2:5)) - repmat(sample,height(avgDB),1)).^2,2)');
    end

end


% ---------------------------subfunctions----------------------------------
function Q = recv(hobj, eventdata, socket, Q)
    while(socket.BytesAvailable > 0)
        data = fgetl(socket);
        Q.enqueue(data); 
    end
end
