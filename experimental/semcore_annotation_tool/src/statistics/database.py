'''
Created on Oct 22, 2012

@author: meyer
'''
import sqlite3
from twisted.spread.jelly import dictionary_atom


class Initialization:
    """
    Class that allows for the creation of new databases.
    """
    
    def get_db(self, name):
        """
        Return a connection handle for a db specified by 'name'.
        """
        
        conn = sqlite3.connect(name)
        return conn
    
    def close_db(self, conn):
        """
        Close the connection to the database.
        """
        
        conn.close()
        
    def finish_transaction(self, conn):
        """
        Finish a transaction by committing.
        """
        conn.commit()
    
    def create_statistics_schema(self, conn):
        """
        Create the schema to save all the data that is necessary to keep 
        statistics about the annotated sentences.
        """
        
        cursor = conn.cursor()
        queries = dict()
        
        queries["dependency"] = """CREATE TABLE dependency(
                                id integer primary key,
                                word_two integer,
                                text integer,
                                sentence integer,
                                type string,
                                word_one integer)"""
        queries["lemma"] = """CREATE TABLE lemma(
                                id integer primary key,
                                nltk_id string)"""
        queries["phrase"] = """CREATE TABLE phrase(
                                id integer primary key,
                                text integer,
                                tag string,
                                sentence integer,
                                type string)"""
        queries["phrase_phrase"] = """CREATE TABLE phrase_phrase(
                                id integer primary key,
                                parent integer,
                                child integer)"""
        queries["sense"] = """CREATE TABLE sense(
                                id integer primary key,
                                wordnet_id string,
                                nltk_ID string)"""
        queries["sentence"] = """CREATE TABLE sentence(
                                id integer primary key,
                                text integer,
                                line_number integer,
                                line_text string)"""
        queries["text"] = """CREATE TABLE text(
                                id integer primary key,
                                title string)"""
        queries["word"] = """CREATE TABLE word(
                                id integer primary key,
                                word string)"""
        queries["word_instance"] = """CREATE TABLE word_instance(
                                id integer primary key,
                                text integer,
                                phrase integer,
                                sentence integer,
                                word integer,
                                position_in_sentence integer,
                                pos string,
                                sense integer,
                                lemma integer)"""
        queries["word_lemma"] = """CREATE TABLE word_lemma(
                                id integer primary key,
                                word integer,
                                lemma integer)"""
        queries["dependency_entity"] = """CREATE TABLE dependency_entity(
                                id integer primary key,
                                type string,
                                text integer)"""
        queries["depentity_phrase"] = """CREATE TABLE depentity_phrase(
                                id integer primary key,
                                depentity integer,
                                phrase integer)"""
        queries["grounding"] = """CREATE TABLE grounding(
                                id integer primary key,
                                phrase integer,
                                object integer)"""
        queries["object"] = """CREATE TABLE object(
                                id integer primary key,
                                object string)"""
        
        map(cursor.execute,
             ["drop table if exists {table_name}".format(table_name = x)
             for x in queries.keys()])
        
        # Delete the old tables
        conn.commit()
        
        map(cursor.execute, queries.values())
        
        conn.commit()
        

class Insertion:
    """
    Class that allows insertion of new items to the database.
    """
    
    def insert_or_update_grounding(self, phrase, objectEntity,
                                     conn=None, cursor=None):
        """
        Fetch an existing ID for a coreference or insert a new coreference into 
        the database.
        """
        
        #The return value
        generated_key = None
        
        if cursor is not None:
            cursor.execute("""select id from grounding where phrase = ? 
                                AND object = ?""", (phrase, objectEntity))
            generated_key = cursor.fetchone()
            
            if generated_key is not None:
                if conn is not None:
                    conn.commit()
                return generated_key[0]
            else:
                cursor.execute("""insert into grounding 
                                    values (null, ?, ?)""",
                                    (phrase, objectEntity))
                
                generated_key = cursor.lastrowid
                if conn is not None:
                    conn.commit()
                return generated_key
            
        return generated_key
    
    def insert_or_update_dependency_entity(self, typ, text, conn=None, 
                                           cursor=None):
        """
        Fetch an existing ID for a dependency_entity or insert a new 
        dependency_entity into the database.
        """
        
        #The return value
        generated_key = None
        
        if cursor is not None:
            cursor.execute("""select id from dependency_entity where text = ? \
                                and type =?""", (text, typ))
            generated_key = cursor.fetchone()
            
            if generated_key is not None:
                if conn is not None:
                    conn.commit()
                return generated_key[0]
            else:
                cursor.execute("""insert into dependency_entity 
                                    values (null, ?, ?)""",
                                    (typ, text))
                
                generated_key = cursor.lastrowid
                if conn is not None:
                    conn.commit()
                return generated_key
            
        return generated_key
    
    def insert_or_update_depentity_phrase(self, dep, phrase, conn=None, 
                                          cursor=None):
        """
        Fetch an existing ID for a depentity_phrase or insert a new 
        depentity_phrase into the database.
        """
        
        #The return value
        generated_key = None
        
        if cursor is not None:
            cursor.execute("""select id from depentity_phrase where 
                                depentity = ? and phrase =?""", (dep, phrase))
            generated_key = cursor.fetchone()
            
            if generated_key is not None:
                if conn is not None:
                    conn.commit()
                return generated_key[0]
            else:
                cursor.execute("""insert into depentity_phrase 
                                    values (null, ?, ?)""",
                                    (dep, phrase))
                
                generated_key = cursor.lastrowid
                if conn is not None:
                    conn.commit()
                return generated_key
            
        return generated_key

    def insert_or_update_object(self, objectEntity, conn=None, cursor=None):
        """
        Fetch an existing ID for an object or insert a new object into 
        the database.
        """
        
        #The return value
        generated_key = None
        
        if cursor is not None:
            cursor.execute("""select id from object where object = ?""",
                           (objectEntity,))
            
            generated_key = cursor.fetchone()
            
            if generated_key is not None:
                if conn is not None:
                    conn.commit()
                return generated_key[0]
            else:
                cursor.execute("""insert into object values (null, ?)""",
                                (objectEntity,))
                generated_key = cursor.lastrowid
                if conn is not None:
                    conn.commit()
                return generated_key
            
        return generated_key
                
    def insert_or_update_text(self, title, conn=None, cursor=None):
        """
        Fetch an existing ID for a text or insert a new text into 
        the database.
        """
        
        #The return value
        generated_key = None
        
        if cursor is not None:
            cursor.execute("""select id from text where title = ?""", (title,))
            generated_key = cursor.fetchone()
            
            if generated_key is not None:
                if conn is not None:
                    conn.commit()
                return generated_key[0]
            else:
                cursor.execute("""insert into text values (null, ?)""",
                                (title,))
                generated_key = cursor.lastrowid
                if conn is not None:
                    conn.commit()
                return generated_key
            
        return generated_key
    
    def insert_or_update_sentence(self, line_number, text, line_text, conn=None,
                                  cursor=None):
        """
        Fetch an existing ID for a sentence or insert a new sentence into 
        the database.
        """
        
        #The return value
        generated_key = None
        
        if cursor is not None:
            cursor.execute("""select id from sentence where line_number = ? 
                            and text = ?""", (line_number, text))
            generated_key = cursor.fetchone()
            
            if generated_key is not None:
                if conn is not None:
                    conn.commit()
                return generated_key[0]
            else:
                cursor.execute("""insert into sentence values (null, ?, ?, ?)"""
                               , (text, line_number, line_text))
                generated_key = cursor.lastrowid
                if conn is not None:
                    conn.commit()
                return generated_key
            
        return generated_key
    
    def insert_or_update_phrase(self, tag, text, sentence, typ, conn=None,
                                cursor=None):
        """
        Fetch an existing ID for a phrase or insert a new phrase into 
        the database.
        """
        
        #The return value
        generated_key = None
        
        if cursor is not None:
            cursor.execute("""select id from phrase where text = ? and 
            sentence = ? and tag = ?""", (text, sentence, tag))
            generated_key = cursor.fetchone()
            
            if generated_key is not None:
                if conn is not None:
                    conn.commit()
                return generated_key[0]
            else:
                cursor.execute("""insert into phrase values 
                (null, ?, ?, ?, ?)""", (text, tag, sentence, typ))
                generated_key = cursor.lastrowid
                if conn is not None:
                    conn.commit()
                return generated_key
            
        return generated_key
    
    def insert_or_update_phrase_phrase(self, parent, child, conn=None,
                                       cursor=None):
        """
        Fetch an existing ID for a phrase relationship or insert a new phrase 
        relationship into the database.
        """
        
        #The return value
        generated_key = None
        
        if cursor is not None:
            cursor.execute("""select id from phrase_phrase where parent = ? 
                            and child = ?""", (parent, child))
            generated_key = cursor.fetchone()
            
            if generated_key is not None:
                if conn is not None:
                    conn.commit()
                return generated_key[0]
            else:
                cursor.execute("""insert into phrase_phrase 
                                values (null, ?, ?)""", (parent, child))
                generated_key = cursor.lastrowid
                if conn is not None:
                    conn.commit()
                return generated_key
            
        return generated_key
    
    def insert_or_update_sense(self, nltkID, conn=None, cursor=None):
        """
        Fetch an existing ID for a sense or insert a new sense 
        into the database.
        """
        
        #The return value
        generated_key = None
        
        if cursor is not None:
            cursor.execute("""select id from sense where nltkID = ?""",
                           (nltkID,))
            generated_key = cursor.fetchone()
            
            if generated_key is not None:
                if conn is not None:
                    conn.commit()
                return generated_key[0]
            else:
                cursor.execute("""insert into sense values (null,?)""",
                               (nltkID,))
                generated_key = cursor.lastrowid
                if conn is not None:
                    conn.commit()
                return generated_key
            
        return generated_key
    
    def insert_or_update_lemma(self, nltkID, conn=None, cursor=None):
        """
        Fetch an existing ID for a lemma or insert a new lemma 
        into the database.
        """
        
        #The return value
        generated_key = None
        
        if cursor is not None:
            cursor.execute("""select id from example where nltkID = ?""",
                           (nltkID,))
            generated_key = cursor.fetchone()
            
            if generated_key is not None:
                if conn is not None:
                    conn.commit()
                return generated_key[0]
            else:
                cursor.execute("""insert into lemma values (null, ?)""",
                               (nltkID,))
                generated_key = cursor.lastrowid
                if conn is not None:
                    conn.commit()
                return generated_key
            
        return generated_key
    
    def insert_or_update_word(self, word, conn=None, cursor=None):
        """
        Fetch an existing ID for a word or insert a new 
        word into the database.
        """
        
        #The return value
        generated_key = None
        
        if cursor is not None:
            cursor.execute("""select id from word where word = ?""", (word,))
            generated_key = cursor.fetchone()
            
            if generated_key is not None:
                if conn is not None:
                    conn.commit()
                return generated_key[0]
            else:
                cursor.execute("""insert into word values (null, ?)""", (word,))
                
                generated_key = cursor.lastrowid
                if conn is not None:
                    conn.commit()
                return generated_key
            
        return generated_key
    
    def insert_or_update_word_instance(self, word, sentence, text, 
                                       position, phrase, pos, lemma, sense,
                                       conn=None, cursor=None):
        """
        Fetch an existing ID for a word_instance or insert a new 
        word_instance into the database.
        """
        
        #The return value
        generated_key = None
        
        if cursor is not None:
            cursor.execute("""select id from word_instance where text = ? 
            and sentence = ? and position_in_sentence = ? and word = ?""",
            (text, sentence, position, word))
            generated_key = cursor.fetchone()
            
            if generated_key is not None:
                if conn is not None:
                    conn.commit()
                return generated_key[0]
            else:
                cursor.execute("""insert into word_instance 
                                values (null, ?, ?, ?, ?, ?, ?, ?, ?)""",
                               (text, phrase, sentence, word, position,
                                pos, sense, lemma))
                
                generated_key = cursor.lastrowid
                if conn is not None:
                    conn.commit()
                return generated_key
            
        return generated_key
    
    def insert_or_update_dependency(self, word1, word2,
                                    typ, text, sentence, conn=None,
                                    cursor=None):
        """
        Fetch an existing id for a dependency or insert a new 
        dependency into the database.
        """
        
        #The return value
        generated_key = None
        
        if cursor is not None:
            cursor.execute("""select id from dependency where word_two = ? 
            and word_one = ? and text = ? and type = ? and sentence = ?""",
            (word2, text, sentence, typ, word1))
            
            generated_key = cursor.fetchone()
            
            if generated_key is not None:
                if conn is not None:
                    conn.commit()
                return generated_key[0]
            else:
                cursor.execute("""insert into dependency values 
                                (null, ?, ?, ?, ?, ?)""",
                               (word2, text, sentence, typ, word1))
                
                generated_key = cursor.lastrowid
                if conn is not None:
                    conn.commit()
                return generated_key
            
        return generated_key


class Modification:
    """
    Class that allows the modification of existing items in the database.
    """
    
    pass

class Deletion:
    """
    Class that allows for deletion of existing items in the database.
    """
    
    pass

if __name__=="__main__":
    initialization = Initialization()
    conn = initialization.get_db('statistics.db')
    c = conn.cursor()
    insert = Insertion()
    #initialization.create_statistics_schema(conn)
    id = insert.insert_or_update_text("Test Title2", cursor=c)
    initialization.finish_transaction(conn)
    initialization.close_db(conn)
    