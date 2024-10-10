classdef Logger
    properties
        
    end

    methods
        function self = Logger()

        end

        function write(self, string)
            
            if strcmp('\erase',string)
                logFileID = fopen('logfile.txt', 'w');
                fprintf(logFileID, '');
                fclose(logFileID);
            else
                logFileID = fopen('logfile.txt', 'a');
                fprintf(logFileID, string);
                fprintf(logFileID, '\n');
                fclose(logFileID);
                disp(string);
            end

        end
    end
end