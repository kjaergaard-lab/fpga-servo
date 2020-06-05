classdef servoCmd < handle
    %SERVOCMD Provides a convenient interface for writing to and reading
    %from commands internal to the servo controllers
    
    properties(Access = public)
        addr        %Hex string indicating the address for serial parsing
        dataType    %Type of data to send to FPGA
        upperLimit  %Upper limit on value
        lowerLimit  %Lower limit on value
    end
    
    properties(Access = protected)
        value               %Human-readable value in real units
        intValue            %Integer value written for FPGA
        device              %Device command is associated with
        toIntegerFunction   %Function converting real values to integer values
        fromIntegerFunction %Function converting integer values to real values
    end
    
    properties(Constant, Hidden = true)
        ADDR_LENGTH = 8;        %Length of address - 8 hex characters
        TYPE_CMD = 'cmd';       %Command type, only address written
        TYPE_INT = 'int';       %Signed integer type
        TYPE_UINT = 'uint';     %Unsigned integer type
        TYPE_ARRAY = 'array';   %Array type - least-significant byte is array index
        TYPE_MEM = 'mem';       %Memory type - written as memory address then data
    end
    
    methods
        function obj = servoCmd(dataType,varargin)
            %FPGACMD Construct fpgaCmd object
            %   c = fpgaCmd(dataType) creates an fpgaCmd object
            %   with the appropriate dataType.
            %   c = fpgaCmd(dataType,'P1','V1','P2','V2',...)
            %   creates an fpgaCmd object with given registers.  Registers
            %   are specified using 'reg0', 'reg1', 'reg2', or 'reg3', and
            %   2 character hex strings 'V1', 'V2', etc.  Can also specify
            %   'addr' and an 8 character hex string
            
            %Check register inputs
            if mod(numel(varargin),2)~=0
                error('You must specify registers as name/value pairs!');
            end
            
            %Set data type
            obj.dataType = dataType;

            %Set the default command to 0
            obj.addr = dec2hex(0,obj.ADDR_LENGTH);
            %set the registers
            obj.setReg(varargin{:});
            %set the conversion functions to default values
            obj.setFunctions('to',@(x) x,'from',@(x) x);
            %Set limits
            obj.upperLimit = [];
            obj.lowerLimit = [];
        end %end fpgaCmd
        
        
        function obj = setReg(obj,varargin)
            %SETREG Sets address and register values
            %   c = setReg('P1','V1','P2','V2',...) sets address or address
            %   registers.  Each parameter can be one of 'addr', 'reg0',
            %   'reg1', 'reg2', or 'reg3'.  For 'addr', the 'V' must be an
            %   8-character hex string.  For each register, 'V' must be a 2
            %   character hex string.
            for nn=1:2:numel(varargin)
                reg = lower(varargin{nn});
                v = varargin{nn+1};
                switch reg
                    case 'addr'
                        obj.addr = v;
                    case 'reg3'
                        obj.addr(1:2) = v;
                    case 'reg2'
                        obj.addr(3:4) = v;
                    case 'reg1'
                        obj.addr(5:6) = v;
                    case 'reg0'
                        obj.addr(7:8) = v;
                    otherwise
                        error('Option %s not supported',reg);
                end
            end
            %Check that address is the correct length
            if numel(obj.addr)~=obj.ADDR_LENGTH
                error('Address must be 8 hex characters long!');
            end
            
            %Check that address can be converted to decimal
            try
                hex2dec(obj.addr);
            catch err
                error('Address must be 8 hex characters long!');
            end
        end %end setReg
        
        function obj = setFunctions(obj,varargin)
            %SETFUNCTIONS Sets the toInteger and fromInteger functions for
            %converting physical values to integer values
            
            %Check register inputs
            if mod(numel(varargin),2)~=0
                error('You must specify functions as name/value pairs!');
            end
            
            for nn=1:2:numel(varargin)
                if ~isa(varargin{nn+1},'function_handle')
                    error('Functions must be passed as function handles!');
                end
                s = lower(varargin{nn});
                switch s
                    case 'to'
                        obj.toIntegerFunction = varargin{nn+1};
                    case 'from'
                        obj.fromIntegerFunction = varargin{nn+1};
                end
            end
        end
        
        function obj = setLimits(obj,varargin)
            %SETLIMITS Sets the upper and lower limits on the physical
            %value
            
            %Check register inputs
            if mod(numel(varargin),2)~=0
                error('You must specify functions as name/value pairs!');
            end
            
            for nn=1:2:numel(varargin)
                s = lower(varargin{nn});
                switch s
                    case 'lower'
                        obj.lowerLimit = varargin{nn+1};
                    case 'upper'
                        obj.upperLimit = varargin{nn+1};
                end
            end
        end
        
        function obj = checkLimits(obj,v)
            %CHECKLIMITS Checks the limits on the set value
            if ~isempty(obj.lowerLimit) && isnumeric(obj.lowerLimit) && (v < obj.lowerLimit)
                error('Value is lower than the lower limit!');
            end
            
            if ~isempty(obj.upperLimit) && isnumeric(obj.upperLimit) && (v > obj.upperLimit)
                error('Value is higher than the upper limit!');
            end
            
        end
        
        function obj = setBits(obj,bits,val)
            %SETBITS Sets individual bits in the address
            %   obj = setBits(bits,val) sets the bits given in the
            %   two-value array bits to the value in val.  Val can be
            %   either a numeric value or a binary string.
            s = hex2bin(obj.addr);
            if isnumeric(val)
                s((end-bits(1)):(end-bits(2))) = dec2bin(val,bits(1)-bits(2)+1);
            elseif ischar(val)
                s((end-bits(1)):(end-bits(2))) = val;
            else
                error('Input value must be either an integer or a character array');
            end
            obj.addr = bin2hex(s);
        end
        
        function set.dataType(obj,dataType)
            %SET.DATATYPE Sets the value of dataType
            dataType = lower(dataType);
            switch dataType
                case obj.TYPE_CMD
                    obj.dataType = obj.TYPE_CMD;
                case obj.TYPE_INT
                    obj.dataType = obj.TYPE_INT;
                case obj.TYPE_UINT
                    obj.dataType = obj.TYPE_UINT;
                case obj.TYPE_ARRAY
                    obj.dataType = obj.TYPE_ARRAY;
                case obj.TYPE_MEM
                    obj.dataType = obj.TYPE_MEM;
                otherwise
                    error('Command type unknown!');
            end
        end %end set.dataType

        function onOff(obj,v)
            %ONOFF Sets the last bit in the addr to 0 or 1.
            if v==0
                obj.addr(end) = '0';
            else
                obj.addr(end) = '1';
            end
        end %end onOff
        
        function obj = setDevice(obj,dev)
            %SETDEVICE Sets the device
            if isa(dev,'servo')
                obj.device = dev;
            else
                error('Device must be of class servo');
            end
        end
        
        function obj = set(obj,v,varargin)
            %SET Sets the physical value of the parameter and converts it
            %to an integer as well
            obj.checkLimits(v);
            obj.value = v;
            obj.intValue = obj.toInteger(obj.value,varargin{:});
            if islogical(obj.intValue)
                obj.intValue = uint32(obj.intValue);
            end
        end
        
        function r = get(obj,varargin)
            %GET Gets the physical value of the parameter from the integer
            %value
            obj.value = obj.fromInteger(obj.intValue,varargin{:});
            r = obj.value;
        end

        function write(obj,devIn)
            %WRITE Writes current integer value data to device.
            %   s.write() writes to device stored in s.device.ser.
            %   s.write(devIn) writes to specified device.
            if nargin==2
                dev = devIn;
            else
                obj.device.open;
                dev = obj.device.ser;
            end
            
            switch obj.dataType
                case obj.TYPE_CMD
                    fwrite(dev,hex2dec(obj.addr),'uint32');
                    
                case obj.TYPE_INT
                    if isnan(obj.intValue(1))
                        error('Value cannot be NaN');
                    end
                    fwrite(dev,hex2dec(obj.addr),'uint32');
                    fwrite(dev,obj.intValue(1),'int32');
                    
                case obj.TYPE_UINT
                    if isnan(obj.intValue(1))
                        error('Value cannot be NaN');
                    end
                    fwrite(dev,hex2dec(obj.addr),'uint32');
                    fwrite(dev,obj.intValue(1),'uint32');
                    
                case obj.TYPE_ARRAY
                    if any(isnan(obj.intValue))
                        error('Values cannot be NaN');
                    end
                    for nn=1:numel(obj.intValue)
                        c = obj.addr;
                        c(7:8) = dec2hex(nn-1,2);
                        fwrite(dev,hex2dec(c),'uint32');
                        fwrite(dev,obj.intValue(nn),'int32');
                    end
                    
                case obj.TYPE_MEM
                    if any(isnan(obj.intValue))
                        error('Values cannot be NaN');
                    end
                    N = numel(obj.intValue);
                    data = zeros(2*N,1,'uint32');
                    data(1:2:end) = uint32(hex2dec(obj.addr) + (0:(N-1))');
                    data(2:2:end) = uint32(obj.intValue(:));
                    %Data is written in blocks because that is faster for
                    %some reason (?)
                    blockSize = 1e2;
                    numBlocks = ceil(numel(data)/blockSize);
                    for nn=1:numBlocks
                        startPos = (nn-1)*blockSize+1;
                        endPos = min(nn*blockSize,numel(data));
                        B = data(startPos:endPos);
                        fwrite(dev,uint32(B(:)),'uint32');
                    end

            end
            pause(1e-3);    %Flushes the event queue
        end %end write
        
        function obj = read(obj,devIn)
            %READ Reads data corresponding to this command
            %   r = read(devIn) reads data from the specified device
            %   r = read() reads data from obj.device.ser.
            if nargin==2
                dev = devIn;
            else
                obj.device.open;
                dev = obj.device.ser;
            end
            
            %Sets the command to read mode
            obj.setBits([29,29],'1');
            
            fwrite(dev,hex2dec(obj.addr),'uint32');
            pause(0.1); %Flush event queue
            %Waits until no more bytes arrive
            b = dev.BytesAvailable;
            pause(0.1);
            while (b~=dev.BytesAvailable)
                b = dev.BytesAvailable;
                pause(0.5);
            end
            r = fread(dev,dev.BytesAvailable/4,'uint32');
            obj.setBits([29,29],'0');   %Resets command to write mode
            %Sets the integer value to what is read out
            obj.intValue = r;  
            try
                obj.get;
            catch err
                warning('Error calling the ''fromInteger'' function.');
            end
        end
        
        function r = toInteger(obj,varargin)
            %TOINTEGER Converts the arguments to an integer
            r = obj.toIntegerFunction(varargin{:});
            try
                r = round(r);
            catch
                
            end
        end
        
        function r = fromInteger(obj,varargin)
            %FROMINTEGER Converts the arguments from an integer
            r = obj.fromIntegerFunction(varargin{:});
        end
        
        function disp(obj)
            fprintf(1,'\t servoCmd with properties:\n');
            fprintf(1,'\t\t                 addr: %s\n',obj.addr);
            fprintf(1,'\t\t             dataType: %s\n',obj.dataType);
            if isnumeric(obj.value) && numel(obj.value)==1
                fprintf(1,'\t\t       Physical value: %.4g\n',obj.value);
            elseif isnumeric(obj.value) && numel(obj.value)<=10
                fprintf(1,'\t\t       Physical value: [%s]\n',strtrim(sprintf('%.4g ',obj.value)));
            elseif isnumeric(obj.value) && numel(obj.value)>10
                fprintf(1,'\t\t       Physical value: [%dx%d %s]\n',size(obj.value),class(obj.value));
            elseif ischar(obj.value)
                fprintf(1,'\t\t       Physical value: %s\n',obj.value);
            end
            if numel(obj.intValue)==1
                fprintf(1,'\t\t        Integer value: %d\n',obj.intValue);
            elseif numel(obj.value)<=10
                fprintf(1,'\t\t        Integer value: [%s]\n',strtrim(sprintf('%d ',obj.intValue)));
            elseif numel(obj.value)>10
                fprintf(1,'\t\t        Integer value: [%dx%d %s]\n',size(obj.value),class(obj.value));
            end
            if ~isempty(obj.lowerLimit) && isnumeric(obj.lowerLimit)
                fprintf(1,'\t\t          Lower limit: %.4g\n',obj.lowerLimit);
            end
            if ~isempty(obj.upperLimit) && isnumeric(obj.upperLimit)
                fprintf(1,'\t\t          Upper limit: %.4g\n',obj.upperLimit);
            end
            
            if ~isempty(obj.toIntegerFunction)
                fprintf(1,'\t\t   toInteger Function: %s\n',func2str(obj.toIntegerFunction));
            end
            if ~isempty(obj.fromIntegerFunction)
                fprintf(1,'\t\t fromInteger Function: %s\n',func2str(obj.fromIntegerFunction));
            end
        end



    end
    
end

function out=hex2bin(str)
    out = dec2bin(hex2dec(str),length(str)*4);
end

function out=bin2hex(str)
    out = dec2hex(bin2dec(str),length(str)/4);
end
